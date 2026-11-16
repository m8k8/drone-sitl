# ドローン制御のSITL練習環境

ArduPilotをMac上で動かし、実機なしで離陸・位置保持・帰還を練習する専用プロジェクトです。最初は標準のクアッドコプターとMAVProxyの文字入力画面を使います。

これはG06の重量・推力・散布装置を再現した物理モデルではありません。操作手順やプログラムの検証用です。

実行・可視化・テスト結果の詳しい資料は、独立した[drone-docsの実験一覧](https://github.com/m8k8/drone-docs/blob/main/sitl/README.md)にまとめています。

## 実験を選ぶ

| 実験 | モデルと目的 | 実行方法 |
| --- | --- | --- |
| [001：標準quad](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/001-standard-quad/README.md) | 3kgの汎用機で環境・基本操作を確認 | 以下の`run.sh`、`smoke-test.sh` |
| [002：G06暫定モデルの積載比較](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/002-g06-payload/README.md) | 公表仕様と仮定から作った機体で、水0・1・3・5Lを比較 | `prepare-g06.sh`、`payload-suite.sh` |

実験002は初回セットアップ後、次の順に実行します。`prepare-g06.sh`は固定版ArduPilotへ同梱の小さなパッチを適用して再ビルドします。機体重量を変えてもモーターの推力が自動的に増えないよう、推進系の校正に使う質量を分離する変更です。

```bash
./scripts/prepare-g06.sh
./scripts/payload-suite.sh --repeats 3
```

各条件の結果・設定・時系列は新しい`runs/g06-payload-*/`に保存します。途中の試験が失敗しても他の条件を続け、全件の成否を記録します。初回に短く動作確認する場合は`./scripts/payload-suite.sh --payloads 0 --repeats 1`を使います。

空荷13.4946kgは未確定部品の仮枠を含む計算上のシナリオです。G06の実測重量や飛行性能を保証するモデルではありません。試験条件・根拠・未検証範囲は[実験002のモデル説明](https://github.com/m8k8/drone-docs/blob/main/sitl/experiments/002-g06-payload/model.md)を参照してください。

`run.sh`の対話起動は引き続き実験001の標準3kg機です。実験002の自動試験は専用インスタンスを使用し、既存の対話起動や実機へ接続しません。

**セットアップとビルドが済んでいる環境では**、`drone`ディレクトリから以下で起動し、「4. 最初の離陸・位置保持・帰還」へ進めます。新しくcloneした場合は「1. セットアップとビルド」から実行してください。

```bash
cd sitl
./scripts/run.sh
```

## 使用する構成

- macOS / Apple Silicon、Xcode Command Line Tools
- HomebrewのPython 3.12と、このディレクトリ専用の`.venv`
- ArduPilot `Copter-4.7.2`
- 固定コミット：`fb773567896014da007bca58358704d4a0d8516f`
- SITL標準quad、MAVProxy 1.8.75 CLI。地図や3D画面は初期構成に含みません。

`requirements-cli.txt`にCLI用のPython依存を固定し、`--no-deps`でその一式を導入します。MAVProxyのGUI・3D用依存は除外しています。

初回はソースとPythonパッケージを取得するため、インターネット接続が必要です。

## 1. セットアップとビルド

別のMacなどで初めて準備する場合の手順です。以下は**macOSのターミナル（シェル）**へ入力します。`drone`ディレクトリを開いた状態から始めてください。

```bash
git clone https://github.com/m8k8/drone-sitl.git sitl
cd sitl
./scripts/setup.sh
./scripts/build.sh
```

既存の`sitl`がある場合はcloneを省略します。

セットアップはArduPilotの取得とPython環境の準備、ビルドはMac用SITL実行ファイルの生成を行います。初回のビルドには時間がかかります。エラーが出たら、その手順の出力を確認してから次へ進んでください。

Pythonや開発ツールが見つからない場合は、先に次のコマンドで場所・バージョンを確認します。

```bash
xcode-select -p
/opt/homebrew/opt/python@3.12/bin/python3.12 --version
```

未導入の場合は`xcode-select --install`でCommand Line Tools、`brew install python@3.12`でPythonを導入します。

## 2. 自動の動作確認

対話操作の前に、シェルで次を実行します。

```bash
./scripts/smoke-test.sh
```

このスクリプトは専用のSITLを起動し、通信の受信、GUIDEDへの変更、アーム、10m離陸、LOITER、RTL、着陸後のディスアームを順に確認します。途中の失敗・タイムアウトは成功扱いにしません。終了コード0が成功です。

各回の記録は`runs/smoke-*/`に保存します。`sitl.log`が機体の起動・動作ログ、`summary.json`が確認結果です。

2026-11-16に、このMac（Apple Silicon、Python 3.12.12、Apple clang 17）でセットアップ、ビルド、自動テストを確認しました。通常のアーム判定を通過し、10m離陸、LOITERの高度・位置保持、RTL着陸と自動ディスアームに成功しています。

このテストの目的は起動環境と基本操作の確認です。AUTO航路、通信断、散布制御、G06の性能を検証したことにはなりません。

## 3. SITLを起動する

シェルで実行します。

```bash
./scripts/run.sh
```

起動後はMAVProxyの入力画面になります。`STABILIZE>`などの表示が入力位置の目印です。通信やGPS・姿勢推定の初期化メッセージが流れるので、初期化が済むまで待ちます。

起動直後に一度だけ`Connection refused sleeping`が出ることがあります。SITL本体の準備を待つ再接続です。その後に`Detected vehicle`や`ArduPilot Ready`が出れば接続できています。繰り返したまま接続できない場合は`/tmp/ArduCopter.log`を確認してください。

ここから先の`mode`、`arm`、`takeoff`は**MAVProxyへ入力するコマンド**です。macOSのシェルには入力しません。説明中の`GUIDED>`などのプロンプトは入力不要です。

## 4. 最初の離陸・位置保持・帰還

### 離陸する

MAVProxyで、まずGUIDEDへ変更します。

```text
mode GUIDED
```

GUIDEDへ切り替わったら、アームします。

```text
arm throttle
```

`ARMED`の通知を確認し、その後10秒以内に次を入力します。2つのコマンドは一括送信せず、アーム完了を見てから離陸を指示します。長く待つと自動でディスアームされることがあります。

```text
takeoff 10
```

アームが拒否された場合は、表示された`PreArm`メッセージを確認します。GPSや姿勢推定の準備中なら待ってから再実行してください。チェックを無効化して進める必要はありません。

### 高度と位置保持を確認する

```text
status GLOBAL_POSITION_INT
```

`relative_alt`は相対高度のミリメートル値です。約`10000`なら、基準高度から約10m上にいます。値は一度だけ表示されるので、必要に応じて再入力します。

高度が安定したら、仮想送信機のスロットルを中立の1500にしてから位置保持へ切り替えます。LOITERではスロットル入力が上昇・下降指示になるため、中立にして高度を保ちます。

```text
rc 3 1500
mode LOITER
```

モードがLOITERになり、しばらく待っても高度が大きく変わらないことを確認します。

### 帰還・着陸する

```text
mode RTL
```

離陸地点へ帰還し、着陸するまで待ちます。RTL設定によっては一度高度を上げてから帰還します。今回の練習では離陸地点の上空にいるため、水平移動がほとんど見られない場合があります。

着陸後に`DISARMED`の通知を確認できれば、この練習は完了です。途中でその場へ着陸させたい場合は、MAVProxyで`mode LAND`を入力します。

### 終了する

ターミナルで`Ctrl+C`を押して停止し、macOSのシェルへ戻ります。

| 記録 | 保存先 |
| --- | --- |
| パラメータ・実行状態 | `runs/default/` |
| 機体の`.BIN`ログ | `runs/default/logs/` |
| MAVProxyの通信ログ | `runs/default/mav.tlog` |
| SITLの起動メッセージ | `/tmp/ArduCopter.log` |

起動には公式の`sim_vehicle.py`を使います。そのheadless起動の仕様により、起動メッセージだけは`/tmp/ArduCopter.log`へ保存され、次回起動時に上書きされます。

## 5. 設定を初期化して再実行する

通常の起動では前回保存したパラメータが使われます。練習を初期状態からやり直す場合は、SITLを停止してからシェルで実行します。

```bash
./scripts/run.sh --reset
```

`--reset`は保存パラメータを初期化する指定です。自分で変更した設定を残したい場合は、通常の`./scripts/run.sh`を使います。

## ディレクトリ構成

```text
sitl/
├── README.md
├── requirements-cli.txt  CLI用Python依存の固定バージョン
├── scripts/          セットアップ・ビルド・起動・動作確認
├── config/           バージョンやシミュレーションの設定
├── vendor/           取得したArduPilotソース（Git管理対象外）
├── .venv/            Python環境（Git管理対象外）
└── runs/             実行状態・パラメータ・ログ（Git管理対象外）
```

最初の練習が終わったら、短いAUTO航路、速度・高度制限、通信断時の動作を一つずつ追加します。手順と期待する結果を記録し、ログで確認していきます。

## リポジトリの管理

公開対象は、この`sitl`プロジェクトのスクリプト・設定・手順書です。`.gitignore`によって、取得したArduPilot本体、Python環境、実行ログを除外します。`drone`内の他の資料はこのプロジェクトへ追加しません。

実行コードは[drone-sitl](https://github.com/m8k8/drone-sitl)、資料は[drone-docs](https://github.com/m8k8/drone-docs)で別々に管理します。変更時は追加対象を確認してコミットします。

```bash
git status --short
git add .
git diff --cached --stat
git commit -m "Describe the change"
```

ArduPilotやPythonパッケージのライセンスは各配布元に従います。このプロジェクトはそれらのソースや配布物をGitへ複製せず、セットアップ時に取得します。

## 公式資料

- [ArduPilot Copter 4.7.2 リリース](https://github.com/ArduPilot/ardupilot/releases/tag/Copter-4.7.2)
- [macOSのビルド環境](https://ardupilot.org/dev/docs/building-setup-mac.html)
- [4.7.2のmacOS依存関係スクリプト](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/Tools/environment_install/install-prereqs-mac.sh)
- [SITLの使い方・設定と保存状態](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
- [Copter / MAVProxy チュートリアル](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html)
- [MAVLink GLOBAL_POSITION_INTの単位](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)
