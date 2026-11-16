# 第１弾B〜DのドローンSITL環境

第１弾の搭載部品表を使うArduPilot SITL専用プロジェクトです。基準重量は電池込み**12.3387kg**。G06、X6 Plus G2×4、12S 10Ah、Pixhawk、M10、下向きTF02-Proなどを想定しています。

現在の機体設定は[`config/prototype-01/`](config/prototype-01/README.md)に集約しています。標準3kg機と旧積載比較の独自コード・設定・結果は削除しました。±0.5kgの2ケースは、同じB〜D機体の重量感度を確認するために残しています。

このモデルは購入前の設計値と仮の部品配置に基づきます。FPV映像、G12の無線伝搬、Remote ID送信、実配線・端子の発熱は再現していません。[モデルの再現範囲と実験結果](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/003-prototype-bd/README.md)を参照してください。

## 1. セットアップとモデルの準備

macOS / Apple Silicon、Xcode Command Line Tools、Python 3.12を使用します。別のMacで初めて準備する場合は、親ディレクトリで次を実行します。

```bash
git clone https://github.com/m8k8/drone-sitl.git sitl
cd sitl
./scripts/setup.sh
./scripts/prepare-prototype.sh
```

既存の`sitl`と`.venv`がある場合はcloneとsetupを省略できます。Pythonが未導入なら`brew install python@3.12`、開発ツールがなければ`xcode-select --install`で準備します。

ArduPilotはCopter 4.7.2、コミット`fb773567896014da007bca58358704d4a0d8516f`に固定しています。`prepare-prototype.sh`は生成ファイルの整合性と既知パッチのハッシュを確認し、推進校正用質量を実際の機体質量から分離する変更を適用してビルドします。`build.sh`単独ではこの準備を行いません。

Python依存は専用`.venv`へ導入します。MAVProxyは1.8.75のCLIを使用し、GUI用依存は含めません。

## 2. 手動で操作する

`drone`からは先に`cd sitl`を実行します。

```bash
./scripts/run.sh
```

**このコマンドもB〜Dの12.3387kg機を起動します。** MAVProxyの文字入力画面で操作します。起動だけでアームや離陸は行いません。

初期化後、MAVProxyへ順番に入力します。以下はmacOSのシェル用コマンドではありません。

```text
mode GUIDED
arm throttle
```

`ARMED`を確認してから、10秒以内に離陸を指示します。

```text
takeoff 10
```

高度が約10mで安定したら、スロットルを中立にして位置保持へ切り替えます。

```text
rc 3 1500
mode LOITER
status GLOBAL_POSITION_INT
```

`relative_alt`はmm単位です。約`10000`なら相対高度10mです。今回のLOITERは下向き測距による地表追従も有効です。

帰還・着陸させます。

```text
mode RTL
```

`DISARMED`を確認後、`Ctrl+C`で終了します。起動したSITLとMAVProxyだけを終了し、他のインスタンスへ停止命令を送りません。アームが拒否された場合はPreArmの理由を確認し、チェックを無効化せず初期化を待ちます。

保存先は`runs/prototype-manual/`です。通常起動は保存パラメータを引き継ぎます。初期状態に戻すにはSITLを停止してから次を実行します。

```bash
./scripts/run.sh --reset
```

`--reset`は`eeprom.bin`だけを初期化し、機体BINログを削除しません。モデル・既定パラメータが変わった場合は、設定の混在を防ぐためリセットを要求します。

手動起動はinstance 41（TCP 6170、RC入力UDP 5911）、自動試験はinstance 40を使用します。使用中ポートへ勝手に接続しません。手動起動のMAVProxyはUDP 14550へ転送するためQGroundControlへ接続できますが、QGCのGUI表示は未検証です。[可視化ガイド](https://github.com/m8k8/drone-docs/blob/main/sitl/guides/visualization.md)

### Gazeboでリアルタイムに見る

Apple SiliconのMacでは、初回に `./scripts/setup-gazebo.sh` を実行します。Gazebo Jettyはプロジェクト内の `.cache/gazebo/` に入ります。次の起動からはこのコマンドだけでGazeboのウィンドウとMAVProxyが開きます。

```bash
./scripts/run.sh --gazebo
```

離陸・帰還は上記のMAVProxy手順で操作します。近くから追うには、Gazeboで機体 `prototype_bd` を右クリックして **Follow** を選びます。離陸後もズームと向きを操作できます。

同じArduCopterと12.3387 kgの `X:model.json` が飛行を計算し、位置と姿勢をローカルUDP 5913経由でGazeboに渡します。機体のモーター位置とプロペラ径はモデルに合わせ、機体外装と飛行場は模式的な描画にしています。Gazeboのウィンドウを閉じても、起動済みのSITLとMAVProxyは動作を続けます。起動ログは各セッションの `gazebo-server.log`、`gazebo-gui.log`、`gazebo-bridge.log` にあります。

## 3. 自動で試験する

```bash
./scripts/prototype-test.sh --scenario flight
./scripts/prototype-test.sh --scenario mission
./scripts/prototype-test.sh --scenario rangefinder
```

| 試験 | 内容 |
| --- | --- |
| `flight` | 通常アーム、10m離陸、LOITER、北20m移動、RTL着陸・通常ディスアーム |
| `mission` | 高度10mで20m四方のAUTO航路、4地点到達、ミッション内RTL |
| `rangefinder` | 正常測距、1mの合成偏りへの応答、範囲外値での高度維持、測距復旧 |

全11件（上記各3回と±0.5kgの基本飛行各1回）を実行する場合は次を使います。

```bash
./scripts/prototype-suite.sh --repeats 3
```

各回は新しい保存状態で起動し、設定・コードのコピー、ハッシュ、判定結果、CSV、BINログを新しい`runs/prototype-*`へ保存します。`--output`で既存ディレクトリを指定した場合は上書きを拒否します。終了コード0が成功です。

2026-11-16の本試験は11/11成功し、`runs/prototype-final-20261116/`に保持しています。成功は記録したモデル・判定条件に対するもので、実機性能の保証ではありません。[試験条件と結果](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/003-prototype-bd/README.md)

### Gazeboで自動試験を見る

Gazebo Jettyのセットアップ後、次のコマンドだけで自動試験用のSITLとGazeboウィンドウが開きます。`run.sh --gazebo` で手動飛行用のウィンドウを開いている場合は、別のウィンドウに試験機が表示されます。

```bash
./scripts/prototype-test.sh --scenario mission --gazebo
```

離陸から帰還・着陸までスクリプトが操作します。`flight` と `rangefinder` にも `--gazebo` を付けられます。シミュレーション速度の既定値は `--gazebo` 付きで1倍速、通常の自動試験で5倍速です。試験用のGazeboは各試験の終了時に閉じます。出力先には判定結果 `summary.json` と描画側のログ `gazebo-*.log` が残ります。

`prototype-suite.sh` でも `--gazebo` を指定できます。各試験が専用のGazeboウィンドウを開きます。

```bash
./scripts/prototype-suite.sh --repeats 1 --scenarios mission --no-mass-sensitivity --gazebo
```

## 4. 保存した結果を見る

```bash
.venv/bin/python scripts/summarize_prototype.py runs/prototype-final-20261116
```

図を再生成する場合だけ、任意の描画用環境を追加します。

```bash
.venv/bin/python -m venv .cache/plots-venv
.cache/plots-venv/bin/python -m pip install -r requirements-plots.txt
.cache/plots-venv/bin/python scripts/plot_prototype.py runs/prototype-final-20261116 \
  --output runs/prototype-final-20261116/figures
```

全件成功したsuiteのAUTO試験3回と測距試験1回目から、軌跡・高度・距離のSVGとPNGを生成します。FPV映像ではありません。公開済みの図は[実験003](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/003-prototype-bd/README.md)にあります。

## ディレクトリ構成

```text
sitl/
├── config/prototype-01/  B〜Dの部品表・モデル・校正・パッチ
├── config/versions.env   ArduPilotの固定版
├── scripts/             準備・手動起動・自動試験・可視化
├── requirements-cli.txt
├── requirements-plots.txt
├── vendor/              ArduPilot依存ソース（Git管理対象外）
├── .venv/               実行用Python（Git管理対象外）
└── runs/                B〜Dの設定状態・実行記録（Git管理対象外）
```

実行コードは[drone-sitl](https://github.com/m8k8/drone-sitl)、部品選定と実験資料は[drone-docs](https://github.com/m8k8/drone-docs)で別々に管理します。生ログ、依存環境、元の購入計画Excelは公開対象に含めません。
