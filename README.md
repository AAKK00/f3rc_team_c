# f3rc_team_c

ESP32 を使った自作ロボットを操作するプログラムです。自動運転と手動操作用のスケッチが含まれています。

## 使用ハードウェア
- ESP32 開発ボード
- VL53L0X 距離センサ
- BNO055 IMU センサ
- Xbox コントローラ (マニュアルモードで使用)
- DCモータおよびアーム用モータ

## 必要なライブラリのインストール
1. Arduino IDE または [arduino-cli](https://arduino.github.io/arduino-cli/).
2. ボードマネージャに ESP32 を追加します。
   “設定 > 追加のボードマネージャ URL” に
   `https://raw.githubusercontent.com/espressif/arduino-esp32/2.0.14/package_esp32_index.json`
   を追加し、"esp32" プラットフォームをインストールします。
3. Arduino IDE のライブラリマネージャから次のライブラリをインストールします:
   - **VL53L0X** (距離センサ用)
   - **Adafruit BNO055**
   - **XboxSeriesXControllerESP32_asukiaaa**
   - (必要に応じて Adafruit Sensor 等)

`arduino-cli` を使う場合は
`arduino-cli lib install "VL53L0X"` などでライブラリをコマンドラインでインストールできます。

## ビルドと書き込み方法
1. 上記ライブラリを完了させたら、Arduino IDE で `2024_f3rc_c_auto.ino` または `2024_f3rc_c_manual.ino` を開きます。
2. ボードは「ESP32 Dev Module」(または使用する ESP32 ボード) を選択します。
3. コンパイル(検証)を行い、問題がなければ、書き込みを実行します。
4. `arduino-cli` を使う場合、例として
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 2024_f3rc_c_auto.ino
   arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32doit-devkit-v1 2024_f3rc_c_auto.ino
   ```
   などを実行します (ポート名は現環境にならって設定)。

## ライセンスと作者
- ライセンス: [MIT License](LICENSE)
- 作者: [AAKK00](https://github.com/AAKK00) (KEIICHI ARIMURA)

