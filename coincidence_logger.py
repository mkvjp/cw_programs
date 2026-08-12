#!/usr/bin/env python3
# coding: utf-8
"""
coincidence_logger.py

coincidencefornano ファームウェアを書き込んだ Arduino Nano Every 4台を
Raspberry Pi 4 の USB に接続し、各ポートのイベントを1つのCSVに記録する。

各ボードは自分の micros() をイベント行に載せて送るので、ホストは定期的に
'T' を送って往復時間の中点でボード時計とホスト時計の対応点を取り、
最小二乗フィット(オフセット+ドリフト)で共通時間軸に変換する。

出力は2ファイル:

イベントCSV (coincidence_日時.csv) — 解析用:
  先頭に「# board <ボード固有名> at <物理ポート位置> (<実デバイス>)」の
  コメント行が入る (単体で対応関係が分かる保険)。
  pandasで読むときは pd.read_csv(f, comment='#') とする。
  est_event_iso : イベント発生時刻のホスト時計換算 (ISO形式)
  est_event_ns  : 同上 (UNIXナノ秒)。コインシデンス探索はこの列同士を比較する
  port          : ボード固有名 (Nano Everyのシリアル番号)。USBの挿し場所を
                  変えても同じボードなら同じ名前になる。/dev/serial/by-id/ が
                  無い環境では /dev/ttyACM0 などにフォールバック
  device_us     : 巻き戻し補正済みのボード時刻 (us)。オフライン再フィット用
  adc           : ADC値

運用ログ (coincidence_日時_log.txt):
  開始/停止/シリアルエラー/再接続の記録。タイムスタンプ付きの1行テキスト。

ポート対応CSV (coincidence_日時_ports.csv) — 開始時に1回書き出す対応表:
  port    : ボード固有名 (イベント/同期CSVのport列と同じ)
  by_id   : /dev/serial/by-id/ のフルネーム (ボード個体に紐づく)
  by_path : /dev/serial/by-path/ の名前 (ラズパイの物理ポート位置に紐づく)
  dev     : 実デバイス (/dev/ttyACM0 など, 起動ごとに変わりうる)

同期CSV (coincidence_日時_sync.csv) — 較正記録・再フィット用の保険:
  host_recv_iso : 同期応答を受信した時刻 (ISO形式)
  host_recv_ns  : 同上 (UNIXナノ秒)
  port          : シリアルポート名
  device_us     : ボード時刻 (us)
  mid_host_ns   : 往復の中点 = device_usに対応するホスト時刻の推定 (UNIXナノ秒)
  rtt_us        : 往復時間 (較正品質の目安)
  used          : 1=較正に使用, 0=RTT大のため除外

使い方:
  pip install pyserial
  python3 coincidence_logger.py                # /dev/ttyACM* を自動検出
  python3 coincidence_logger.py --ports /dev/ttyACM0 /dev/ttyACM1 ...
  python3 coincidence_logger.py --hours 24     # 24時間で自動停止 (既定は無制限)
"""

import argparse
import collections
import csv
import glob
import os
import queue
import re
import sys
import threading
import time

import serial

BAUD = 115200
SYNC_INTERVAL_S = 2.0      # 'T' を送る間隔 (Nano Every は内蔵RC発振で~0.5%ドリフトするので短め)
STARTUP_SYNCS = 6          # 起動直後はこの回数だけ0.5s間隔で密に同期する
STARTUP_SYNC_INTERVAL_S = 0.5
BOOT_WAIT_S = 2.5          # ポートを開くとボードがリセットされるので待つ
RECONNECT_INTERVAL_S = 3.0 # USB切断時に再接続を試みる間隔
SYNC_TIMEOUT_NS = 2_000_000_000  # 同期応答をこれ以上待ったら諦めて次を送る
MICROS_WRAP = 2 ** 32      # micros() の一周
CAL_POINTS = 10            # フィットに使う直近の同期点数
RTT_ACCEPT_NS = 4_000_000  # RTTがこれ(4ms)を超える同期点は較正に使わない
STALE_ACCEPT_NS = 10_000_000_000  # ただし10s以上採用点がない場合はRTTが大きくても受け入れる


def find_ports():
    """ボード固有名(/dev/serial/by-id)を優先し、無ければ/dev/ttyACM*を使う"""
    ports = sorted(glob.glob('/dev/serial/by-id/usb-*'))
    if ports:
        return ports
    return sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))


def board_label(port):
    """/dev/serial/by-id/usb-Arduino_..._A1B2C3D4-if00 -> A1B2C3D4 (シリアル番号)"""
    base = os.path.basename(port)
    if base.startswith('usb-'):
        base = re.sub(r'-if\d+$', '', base[4:])
        return base.split('_')[-1]
    return port  # /dev/ttyACM0 などはそのまま


def by_path_name(port):
    """同じデバイスを指す /dev/serial/by-path/ の名前 (ラズパイの物理ポート位置)"""
    real = os.path.realpath(port)
    for p in glob.glob('/dev/serial/by-path/*'):
        if os.path.realpath(p) == real:
            return os.path.basename(p)
    return ''


class ClockModel:
    """ボード時刻(us) -> ホスト時刻(ns) の線形変換 (最小二乗)"""

    def __init__(self):
        self.points = collections.deque(maxlen=CAL_POINTS)  # (device_us, host_ns)

    def add(self, device_us, host_ns):
        self.points.append((device_us, host_ns))

    def estimate(self, device_us):
        n = len(self.points)
        if n == 0:
            return None
        if n == 1:
            d0, h0 = self.points[0]
            return h0 + (device_us - d0) * 1000
        # 数値安定のため最初の点を原点にして最小二乗
        d0, h0 = self.points[0]
        xs = [d - d0 for d, _ in self.points]
        ys = [h - h0 for _, h in self.points]
        sx = sum(xs); sy = sum(ys)
        sxx = sum(x * x for x in xs); sxy = sum(x * y for x, y in zip(xs, ys))
        denom = n * sxx - sx * sx
        if denom == 0:
            return h0 + (device_us - d0) * 1000
        a = (n * sxy - sx * sy) / denom          # ns per us (理想は1000)
        b = (sy - a * sx) / n
        return int(h0 + a * (device_us - d0) + b)


class PortWorker(threading.Thread):
    """1ポート分: 読み取り + micros巻き戻し補正 + 時計較正"""

    def __init__(self, port, out_queue, stop_event):
        super().__init__(daemon=True, name=port)
        self.port = port
        self.label = board_label(port)
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.ser = None
        self.wraps = 0
        self.last_micros = None
        self.clock = ClockModel()
        self.pending_sync_ns = None
        self.last_used_ns = None
        self.lock = threading.Lock()

    def open(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=1)

    def send_sync(self):
        ser = self.ser
        if ser is None:
            return  # 切断中
        now = time.time_ns()
        with self.lock:
            # 前回の応答待ち (ただし応答が失われた場合に備えてタイムアウト付き)
            if (self.pending_sync_ns is not None
                    and now - self.pending_sync_ns < SYNC_TIMEOUT_NS):
                return
            self.pending_sync_ns = now
        try:
            ser.write(b'T')
        except (serial.SerialException, OSError):
            with self.lock:
                self.pending_sync_ns = None

    def reset_clock_state(self):
        """再接続時に呼ぶ。ポートを開き直すとボードがリセットされて
        micros()が0からやり直しになるので、時計関連の状態を全て初期化する"""
        self.wraps = 0
        self.last_micros = None
        self.clock = ClockModel()
        self.last_used_ns = None
        with self.lock:
            self.pending_sync_ns = None

    def reconnect(self):
        """切断されたポートに再接続できるまで試み続ける。停止要求でFalse"""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        while not self.stop_event.is_set():
            self.stop_event.wait(RECONNECT_INTERVAL_S)
            try:
                ser = serial.Serial(self.port, BAUD, timeout=1)
                time.sleep(BOOT_WAIT_S)  # 開くとボードがリセットされるので起動を待つ
                ser.reset_input_buffer()
            except (serial.SerialException, OSError):
                continue
            self.reset_clock_state()
            self.ser = ser
            self.log('再接続しました')
            return True
        return False

    def log(self, msg):
        """運用ログ: コンソールとログファイル(キュー経由)の両方に残す"""
        print(f'[{self.label}] {msg}', file=sys.stderr)
        self.out_queue.put({'type': 'L', 'host_recv_ns': time.time_ns(),
                            'msg': f'[{self.label}] {msg}'})

    def unwrap(self, m):
        if self.last_micros is not None and m < self.last_micros - MICROS_WRAP // 2:
            self.wraps += 1
        self.last_micros = m
        return m + self.wraps * MICROS_WRAP

    def run(self):
        while not self.stop_event.is_set():
            if self.ser is None:
                if not self.reconnect():
                    return
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.log(f'シリアルエラー: {e} -> 再接続を試みます')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue
            recv_ns = time.time_ns()
            if not line:
                continue
            try:
                text = line.decode('ascii').strip()
            except UnicodeDecodeError:
                continue
            parts = text.split(',')

            if parts[0] == 'E' and len(parts) == 3:
                try:
                    m = int(parts[1]); adc = int(parts[2])
                except ValueError:
                    continue
                device_us = self.unwrap(m)
                est = self.clock.estimate(device_us)
                if est is None:
                    est = recv_ns  # 較正前(開始直後~0.5s)のみ受信時刻で代用
                self.out_queue.put({
                    'type': 'E', 'est_event_ns': est, 'port': self.label,
                    'device_us': device_us, 'adc': adc,
                })

            elif parts[0] == 'S' and len(parts) == 2:
                try:
                    m = int(parts[1])
                except ValueError:
                    continue
                device_us = self.unwrap(m)
                with self.lock:
                    t_send = self.pending_sync_ns
                    self.pending_sync_ns = None
                if t_send is None:
                    continue
                rtt_ns = recv_ns - t_send
                mid_ns = t_send + rtt_ns // 2
                # RTTが大きい同期点は中点の不確かさも大きいので較正から除外
                # (較正点がまだ少ない/しばらく採用がない場合は受け入れる)
                stale = (self.last_used_ns is None
                         or recv_ns - self.last_used_ns > STALE_ACCEPT_NS)
                used = rtt_ns <= RTT_ACCEPT_NS or len(self.clock.points) < 2 or stale
                if used:
                    self.clock.add(device_us, mid_ns)
                    self.last_used_ns = recv_ns
                self.out_queue.put({
                    'type': 'S', 'host_recv_ns': recv_ns, 'port': self.label,
                    'device_us': device_us, 'mid_host_ns': mid_ns,
                    'rtt_us': rtt_ns // 1000, 'used': int(used),
                })


def iso(ns):
    if ns == '' or ns is None:
        return ''
    t = ns / 1e9
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t)) + f'.{ns % 1_000_000_000:09d}'


def main():
    ap = argparse.ArgumentParser(description='4台のNano Everyのイベントを1つのCSVに記録')
    ap.add_argument('--ports', nargs='+', help='使用するポート (省略時 /dev/ttyACM* を自動検出)')
    ap.add_argument('--out', help='出力CSVファイル名 (省略時 coincidence_日時.csv)')
    ap.add_argument('--hours', type=float, default=0.0, help='記録時間[時間] (0で無制限, 既定は無制限)')
    args = ap.parse_args()

    ports = args.ports or find_ports()
    if not ports:
        print('シリアルポートが見つかりません', file=sys.stderr)
        sys.exit(1)
    print('使用ポート:')
    for p in ports:
        print(f'  {board_label(p)}  ({p})')

    out_name = args.out or time.strftime('coincidence_%Y%m%d_%H%M%S.csv')
    base = out_name[:-4] if out_name.endswith('.csv') else out_name
    sync_name = base + '_sync.csv'
    ports_name = base + '_ports.csv'
    log_name = base + '_log.txt'

    # ボード固有名・物理ポート位置・実デバイスの対応表を書き出す
    with open(ports_name, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['port', 'by_id', 'by_path', 'dev'])
        for p in ports:
            is_by_id = '/by-id/' in p
            w.writerow([board_label(p),
                        os.path.basename(p) if is_by_id else '',
                        by_path_name(p),
                        os.path.realpath(p)])
        f.flush()
        os.fsync(f.fileno())
    stop_event = threading.Event()
    q = queue.Queue()

    workers = [PortWorker(p, q, stop_event) for p in ports]
    for w in workers:
        try:
            w.open()
        except (serial.SerialException, OSError) as e:
            w.log(f'接続できません: {e} -> バックグラウンドで再接続を試みます')
            w.ser = None
    print(f'ボードのリセット待ち {BOOT_WAIT_S}s ...')
    time.sleep(BOOT_WAIT_S)
    for w in workers:
        if w.ser is not None:
            w.ser.reset_input_buffer()
        w.start()

    deadline = time.time() + args.hours * 3600 if args.hours > 0 else None

    event_fields = ['est_event_iso', 'est_event_ns', 'port', 'device_us', 'adc']
    sync_fields = ['host_recv_iso', 'host_recv_ns', 'port',
                   'device_us', 'mid_host_ns', 'rtt_us', 'used']
    n_events = 0
    n_syncs = 0
    last_sync = 0.0
    last_fsync = 0.0
    try:
        with open(out_name, 'w', newline='') as f_ev, \
             open(sync_name, 'w', newline='') as f_sy, \
             open(log_name, 'w') as f_log:

            def write_log(msg, ts=None):
                f_log.write(f'{iso(ts if ts is not None else time.time_ns())} {msg}\n')
                f_log.flush()

            # ボードと物理ポートの対応をコメント行として先頭に残す (保険)
            for p in ports:
                f_ev.write(f'# board {board_label(p)} at {by_path_name(p) or "?"}'
                           f' ({os.path.realpath(p)})\n')
            ev_writer = csv.DictWriter(f_ev, fieldnames=event_fields)
            ev_writer.writeheader()
            sy_writer = csv.DictWriter(f_sy, fieldnames=sync_fields)
            sy_writer.writeheader()
            print(f'記録開始 -> {out_name} / {sync_name} (Ctrl+Cで停止)')
            write_log('記録開始 ' + ' '.join(board_label(p) for p in ports))
            while True:
                now = time.time()
                if deadline and now >= deadline:
                    print('指定時間に達したので停止します')
                    write_log('指定時間に達したので停止')
                    break
                # 同時送信するとUSBバス上で応答が競合してRTTが膨らむので、
                # ポートごとに時間をずらして1台ずつ同期する (ラウンドロビン)。
                # 起動直後は密に同期してドリフト(傾き)を早く確定させる。
                period = (STARTUP_SYNC_INTERVAL_S
                          if n_syncs < STARTUP_SYNCS * len(workers) else SYNC_INTERVAL_S)
                if now - last_sync >= period / len(workers):
                    last_sync = now
                    workers[n_syncs % len(workers)].send_sync()
                    n_syncs += 1
                try:
                    row = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                kind = row.pop('type')
                if kind == 'E':
                    row['est_event_iso'] = iso(row['est_event_ns'])
                    ev_writer.writerow(row)
                    f_ev.flush()
                    n_events += 1
                    if n_events % 100 == 0:
                        print(f'イベント {n_events} 件')
                elif kind == 'S':
                    row['host_recv_iso'] = iso(row['host_recv_ns'])
                    sy_writer.writerow(row)
                    f_sy.flush()
                else:  # 'L': 切断・再接続などの運用ログ
                    write_log(row['msg'], ts=row['host_recv_ns'])
                # 電源断対策: 1秒に1回ディスクへ強制書き込み
                if now - last_fsync >= 1.0:
                    last_fsync = now
                    os.fsync(f_ev.fileno())
                    os.fsync(f_sy.fileno())
                    os.fsync(f_log.fileno())
    except KeyboardInterrupt:
        print('\n停止します')
        try:
            with open(log_name, 'a') as f_log:
                f_log.write(f'{iso(time.time_ns())} Ctrl+Cで停止\n')
        except OSError:
            pass
    finally:
        stop_event.set()
        for w in workers:
            if w.ser:
                try:
                    w.ser.close()
                except Exception:
                    pass
        print(f'イベント合計 {n_events} 件 -> {out_name}')


if __name__ == '__main__':
    main()
