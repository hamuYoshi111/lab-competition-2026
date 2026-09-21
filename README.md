# orienteering-competition2026

正門を発着点として、候補地点から6〜12か所を選び、巡回順を求めるプログラムです。
配布された遺伝的アルゴリズムに局所探索を加え、地点数・地点間隔・所要時間・累積登りから計算する評価値を小さくします。

## ビルドと実行

C++17に対応したコンパイラが必要です。`input/` のある、このフォルダーで実行してください。

### macOS / Linux

```sh
make
./orienteering
```

### Windows

Visual Studio 2022 Build Toolsを既定の場所にインストールした環境では、次の手順で実行できます。

```bat
build.bat
orienteering.exe
```

Visual Studioでプロジェクトを作る場合は、このフォルダーの `.cpp` 5本と `.h` 5本を追加し、C++17・Release構成でビルドしてください。

## 入力と出力

入力は `input/` 内の `landmarks.csv`、`nodes.csv`、`edges.csv`、`seimon.csv` です。
実行すると `output/` が作られ、コースを `best_course.json`、世代ごとの評価値を `fitness_history.csv` に保存します。

引数なしでは、乱数の種42・個体数10・世代数20で探索を3回行い、最良のコースを採用します。
種を変える場合は `./orienteering 7`（Windowsでは `orienteering.exe 7`）のように指定できます。
画面の「実行時間」は探索部分、「総実行時間」は読み込み・事前計算・出力も含む時間です。
