// coincidencefornano.ino
// 4台同時計測(コインシデンス測定)用ファームウェア
//
// 出力プロトコル (115200 baud):
//   イベント行 : E,<micros>,<adc>   ... adc >= 100 のとき送信
//   同期応答行 : S,<micros>         ... ホストから 'T' を受信した直後に送信
//
// micros() は約71.6分で一周(2^32 us)するが、巻き戻しはホスト側で補正する。

void setup() {
  analogReference(EXTERNAL);
  Serial.begin(115200);
  delay(900);
}

void loop() {
  // ホストからの時刻同期要求 ('T') に即応答する
  while (Serial.available()) {
    if (Serial.read() == 'T') {
      unsigned long t = micros();
      Serial.print(F("S,"));
      Serial.println(t);
    }
  }

  int adc = analogRead(A0);
  if (adc >= 100) {
    unsigned long t = micros();
    Serial.print(F("E,"));
    Serial.print(t);
    Serial.print(',');
    Serial.println(adc);
  }
}
