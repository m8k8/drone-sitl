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
./scripts/prototype-test.sh --scenario field
./scripts/prototype-test.sh --scenario sloped_field
./scripts/prototype-test.sh --scenario stepped_field
./scripts/prototype-test.sh --scenario planned_step_crossing
./scripts/prototype-test.sh --scenario battery_failsafe
./scripts/prototype-test.sh --scenario battery_failsafe_critical
./scripts/prototype-test.sh --scenario rangefinder
```

| 試験 | 内容 |
| --- | --- |
| `flight` | 通常アーム、10m離陸、LOITER、北20m移動、RTL着陸・通常ディスアーム |
| `mission` | 高度10mで20m四方のAUTO航路、4地点到達、ミッション内RTL |
| `field` | 平坦な圃場の3列を高度10mでジグザグに飛ぶAUTO航路、6地点到達、ミッション内RTL |
| `sloped_field` | 緩い傾斜のある圃場をAUTOで上り下りし、対地高度10 mを保てるかを試験 |
| `stepped_field` | 高さ1 mの段差をAUTOで3回横切り、最低対地高度と10 m付近への回復時間を測定 |
| `planned_step_crossing` | 高さ10 mの段差を事前に高度を設定したAUTO航路で往復し、通過時の対地高度を測定 |
| `battery_failsafe` | AUTO飛行中の低電圧でRTLに切り替わり、離陸地点に帰還・着陸するかを試験 |
| `battery_failsafe_critical` | RTL中にさらに電圧が下がるとLANDに切り替わり、その地点付近に着陸するかを試験 |
| `rangefinder` | 正常測距、1mの合成偏りへの応答、範囲外値での高度維持、測距復旧 |

実験003と同じ11件（`flight`、`mission`、`rangefinder`を各3回、±0.5kgの基本飛行を各1回）を実行する場合は次を使います。

```bash
./scripts/prototype-suite.sh --repeats 3
```

各回は新しい保存状態で起動し、設定・コードのコピー、ハッシュ、判定結果、CSV、BINログを新しい`runs/prototype-*`へ保存します。`--output`で既存ディレクトリを指定した場合は上書きを拒否します。終了コード0が成功です。

2026-11-16の本試験は11/11成功し、`runs/prototype-final-20261116/`に保持しています。成功は記録したモデル・判定条件に対するもので、実機性能の保証ではありません。[試験条件と結果](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/003-prototype-bd/README.md)

### 平坦な圃場の往復航路

`field`は既定で無風の条件を使い、圃場脇から離陸したあと、基準機が離陸地点からの相対高度10mで3列をジグザグに飛びます。1列は24m、列間は8mです。各地点で2秒待機した後、最後の指示で離陸地点へ帰還し、着陸してディスアームします。6地点の到達と着地位置は実行先の`summary.json`で、飛行軌跡は`telemetry.csv`で確認できます。

基準機でこのシナリオを繰り返す場合は、次を使います。

```bash
./scripts/prototype-suite.sh --scenarios field --no-mass-sensitivity
```

### 緩い傾斜のある圃場の対地高度試験

`sloped_field`では、北へ20 m進むごとに地面が1 m高くなる斜面をAUTOで上り下りします。航路の目標は対地高度10 mで、6地点への飛行中は目標からのずれが±0.5 m以内なら高度の判定に合格します。SITLの模擬測距に合成地形を反映し、機体の真値高度から真下の地表高度を引いて対地高度を計算します。実行先の `summary.json` に判定と高度の範囲、`terrain-following.csv` に対地高度と測距の時系列を保存します。

```bash
./scripts/prototype-test.sh --scenario sloped_field
```

斜面と飛行の様子をGazeboで見る場合は、Gazebo Jettyをセットアップしてから次を実行します。

```bash
./scripts/prototype-test.sh --scenario sloped_field --gazebo
```

### 段差のある圃場の対地高度試験

`stepped_field`では、地面が水平距離1 mの間に1 m高くなる段差を、目標対地高度10 mで上り、下り、上りの順に横切ります。各通過で機体の真下の地表から測った最低高度と、10 m付近へ戻るまでの時間を記録します。

回復時間は、段差の移行区間に入ってから、通過後の対地高度が9.75〜10.25 mに1秒以上とどまった区間の始まりまでです。次の通過またはミッション終了までに回復しなければタイムアウトとします。`summary.json` に通過ごとの測定値と判定、`terrain-following.csv` に対地高度と測距の時系列を保存します。

```bash
./scripts/prototype-test.sh --scenario stepped_field
./scripts/prototype-test.sh --scenario stepped_field --gazebo
```

### 高さ10 mの段差を越える計画航路

`planned_step_crossing`では、地面が水平距離1 mの間に10 m高くなる圃場を、離陸地点からの相対高度を指定した6地点のAUTO航路で往復します。段差の手前で相対高度22 mまで上がり、段差を越えた直後は地面から約12 mを確保する計画です。高い側の点検地点では相対高度20 m、対地高度10 mで飛びます。帰路でも相対高度22 mで段差を越え、低い側で相対高度10 mに下がってからRTLで帰還します。

段差の移行区間とその前後2 mで、対地高度11 m以上を合格条件にします。高い側の点検地点は10 m±0.5 mで判定します。`summary.json` に往復の最低対地高度と試験の判定、`terrain-following.csv` に飛行中の地表高度・対地高度・有効な測距の時系列を保存します。

```bash
./scripts/prototype-test.sh --scenario planned_step_crossing
./scripts/prototype-test.sh --scenario planned_step_crossing --gazebo
```

### 圃場のバッテリーフェイルセーフ試験

平坦な圃場でAUTO飛行中にSITLの模擬バッテリー電圧を下げ、ArduCopterがバッテリーフェイルセーフでモードを切り替えるかを確認します。`battery_failsafe` は低電圧でRTLに切り替わり、離陸地点のパッドへ帰還・着陸するケースです。`battery_failsafe_critical` はRTLで帰還している途中にさらに電圧を下げ、LANDに切り替わった地点付近で着陸するケースです。

この試験では、負荷がかかった状態の電圧が42.0 V未満で2秒続くと低電圧、40.8 V未満で2秒続くと危険域と判定します。実行先の `summary.json` には機体ログで確認した切り替え理由と着陸位置の判定を、`telemetry.csv` には電圧と飛行モードの時系列を保存します。圃場と機体の動きは、それぞれの実行コマンドに `--gazebo` を付けると表示できます。

```bash
./scripts/prototype-test.sh --scenario battery_failsafe
./scripts/prototype-test.sh --scenario battery_failsafe_critical
./scripts/prototype-test.sh --scenario battery_failsafe --gazebo
./scripts/prototype-test.sh --scenario battery_failsafe_critical --gazebo
```

### Gazeboで自動試験を見る

Gazebo Jettyのセットアップ後、次のコマンドだけで自動試験用のSITLとGazeboウィンドウが開きます。`run.sh --gazebo` で手動飛行用のウィンドウを開いている場合は、別のウィンドウに試験機が表示されます。

```bash
./scripts/prototype-test.sh --scenario field --gazebo
```

`field`では圃場と離着陸地点を表示します。Gazeboは表示専用で、飛行・測距・接地はSITLが計算します。`flight`、`mission`、`rangefinder`にも`--gazebo`を付けられます。シミュレーション速度の既定値は `--gazebo` 付きで1倍速、通常の自動試験で5倍速です。試験用のGazeboは各試験の終了時に閉じます。出力先には判定結果 `summary.json` と描画側のログ `gazebo-*.log` が残ります。

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
│   └── scenarios/       シナリオごとの試験手順と共通AUTO処理
├── requirements-cli.txt
├── requirements-plots.txt
├── vendor/              ArduPilot依存ソース（Git管理対象外）
├── .venv/               実行用Python（Git管理対象外）
└── runs/                B〜Dの設定状態・実行記録（Git管理対象外）
```

各試験の手順は `scripts/scenarios/<シナリオ名>/scenario.py` にあります。`scripts/scenarios/auto.py` に航路の登録・実行に使う共通処理があります。圃場のGazebo表示は `scripts/scenarios/` 配下の表示コードで定義します。自動試験は使用したシナリオのコードを実行先の `scenarios/` に保存します。

実行コードは[drone-sitl](https://github.com/m8k8/drone-sitl)、部品選定と実験資料は[drone-docs](https://github.com/m8k8/drone-docs)で別々に管理します。生ログ、依存環境、元の購入計画Excelは公開対象に含めません。
