#!/usr/bin/env python3
# coding: utf-8
"""
coincidence_logger.py

coincidencefornano ファームウェアを書き込んだ Arduino Nano Every 4台を
Raspberry Pi 4 の USB に接続し、各ポートのイベントを1つのCSVに記録する。

各ボードは自分の micros() をイベント行に載せて送るので、ホストは定期的に
'T' を送って往復時間の中点でボード時計とホスト時計の対応点を取り、
最小二乗フィット(オフセット+ドリフト)で共通時間軸に変換する。

CSVの列:
  host_recv_iso   : ホストが行を受信した時刻 (ISO形式)
  host_recv_ns    : 同上 (UNIXナノ秒)
  port            : シリアルポート名 (/dev/ttyACM0 など)
  type            : E=イベント, S=同期応答
  device_micros   : ボードのmicros()生値
  device_us       : 巻き戻し補正済みのボード時刻 (us)
  adc             : ADC値 (イベント行のみ)
  est_event_ns    : イベント発生時刻のホスト時計換算 (UNIXナノ秒, 較正済み)
  sync_rtt_us     : 同期の往復時間 (同期行のみ, 較正品質の目安)

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
import sys
import threading
import time

import serial

BAUD = 115200
SYNC_INTERVAL_S = 10.0     # 'T' を送る間隔
BOOT_WAIT_S = 2.5          # ポートを開くとボードがリセットされるので待つ
MICROS_WRAP = 2 ** 32      # micros() の一周
CAL_POINTS = 6             # フィットに使う直近の同期点数


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
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.ser = None
        self.wraps = 0
        self.last_micros = None
        self.clock = ClockModel()
        self.pending_sync_ns = None
        self.lock = threading.Lock()

    def open(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=1)

    def send_sync(self):
        with self.lock:
            if self.pending_sync_ns is not None:
                return  # 前回の応答待ち
            self.pending_sync_ns = time.time_ns()
        try:
            self.ser.write(b'T')
        except serial.SerialException:
            with self.lock:
                self.pending_sync_ns = None

    def unwrap(self, m):
        if self.last_micros is not None and m < self.last_micros - MICROS_WRAP // 2:
            self.wraps += 1
        self.last_micros = m
        return m + self.wraps * MICROS_WRAP

    def run(self):
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
            except serial.SerialException as e:
                print(f"[{self.port}] シリアルエラー: {e}", file=sys.stderr)
                break
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
                self.out_queue.put({
                    'host_recv_ns': recv_ns, 'port': self.port, 'type': 'E',
                    'device_micros': m, 'device_us': device_us,
                    'adc': adc, 'est_event_ns': est, 'sync_rtt_us': '',
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
                self.clock.add(device_us, mid_ns)
                self.out_queue.put({
                    'host_recv_ns': recv_ns, 'port': self.port, 'type': 'S',
                    'device_micros': m, 'device_us': device_us,
                    'adc': '', 'est_event_ns': mid_ns,
                    'sync_rtt_us': rtt_ns // 1000,
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

    ports = args.ports or sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print('シリアルポートが見つかりません', file=sys.stderr)
        sys.exit(1)
    print(f'使用ポート: {", ".join(ports)}')

    out_name = args.out or time.strftime('coincidence_%Y%m%d_%H%M%S.csv')
    stop_event = threading.Event()
    q = queue.Queue()

    workers = [PortWorker(p, q, stop_event) for p in ports]
    for w in workers:
        w.open()
    print(f'ボードのリセット待ち {BOOT_WAIT_S}s ...')
    time.sleep(BOOT_WAIT_S)
    for w in workers:
        w.ser.reset_input_buffer()
        w.start()

    deadline = time.time() + args.hours * 3600 if args.hours > 0 else None

    fields = ['host_recv_iso', 'host_recv_ns', 'port', 'type',
              'device_micros', 'device_us', 'adc', 'est_event_iso', 'est_event_ns', 'sync_rtt_us']
    n_events = 0
    last_sync = 0.0
    last_fsync = 0.0
    try:
        with open(out_name, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            print(f'記録開始 -> {out_name} (Ctrl+Cで停止)')
            while True:
                now = time.time()
                if deadline and now >= deadline:
                    print('指定時間に達したので停止します')
                    break
                if now - last_sync >= SYNC_INTERVAL_S:
                    last_sync = now
                    for w in workers:
                        w.send_sync()
                try:
                    row = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                row['host_recv_iso'] = iso(row['host_recv_ns'])
                row['est_event_iso'] = iso(row['est_event_ns'])
                writer.writerow(row)
                f.flush()
                # 電源断対策: 1秒に1回ディスクへ強制書き込み
                if now - last_fsync >= 1.0:
                    last_fsync = now
                    os.fsync(f.fileno())
                if row['type'] == 'E':
                    n_events += 1
                    if n_events % 100 == 0:
                        print(f'イベント {n_events} 件')
    except KeyboardInterrupt:
        print('\n停止します')
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
