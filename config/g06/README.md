# G06の暫定荷重モデル

この設定は、公表仕様と明示した仮定から作る荷重比較用モデルです。実機の飛行可否・飛行時間を保証するモデルではありません。仕様、仮定、誤差は[実験002のモデル説明](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/002-g06-payload/model.md)を参照してください。

```bash
./scripts/prepare-g06.sh
```

固定版ArduPilotの既知の2ファイルだけに`reference-mass.patch`を適用してビルドします。既存の変更や途中まで適用されたパッチを検出した場合は上書きせず停止します。繰り返し実行できます。標準モデルは`referenceMass`未指定時に従来どおりの計算を使います。

`models/payload-{0,1,3,5}l.json`では、実質量だけを変え、推進系を校正する`referenceMass`を13.4946kgに固定します。`defaults.parm`は公式`copter.parm`の後から読み込むSITL専用設定です。

```bash
.venv/bin/python config/g06/generate.py --check
```

これで生成ファイル、校正誤差、全荷重で推進係数が一致することを確認できます。`--check`なしで実行すると、生成モデル・manifest・calibration・パッチを再生成します。実機には適用しないでください。

`reference-mass.patch`は[ArduPilotの固定版](https://github.com/ArduPilot/ardupilot/tree/fb773567896014da007bca58358704d4a0d8516f)の`SIM_Frame.h/.cpp`を変更するGPL-3.0-or-laterの差分です。ArduPilotの著作権表示と[ライセンス](https://github.com/ArduPilot/ardupilot/blob/fb773567896014da007bca58358704d4a0d8516f/COPYING.txt)に従います。
