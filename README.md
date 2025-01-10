# One-Stroke Robot Project @2024 team B3

## 🎯 **プロジェクトの目的**
名古屋大学 機械航空宇宙工学科3年次の設計製図第3 一筆書きプロジェクトです．
このプロジェクトでは、課題として提示された図形を **速く**、**正確に**、そして **一筆書きで描く** ことを目的としたロボット，およびその開発プラットフォームを開発しました．
![デモGIF](https://github.com/LaLaSero/one-stroke/raw/main/simulator/robot_arm_simulation.gif)
---

## 🛠 **ハードウェア構成**
本ロボットの構成は以下の通りです：

- **制御マイコン**: Arduino Mega 2560（互換品）
- **駆動部**: モーターユニット × 2
- **リンク機構**: 描画アーム（リンク付き）

---

## 🧠 **ソフトウェア構成**
制御プログラムを開発するためのプラットフォームは、以下の3つの主要モジュールで構成されています：

### 1️⃣ **パス生成プログラム**
- **概要**: 与えられた図形データをもとに、描画用の経路（path）を計算・生成します。
- **特徴**: pythonを使用することで，例えば図形の一部分だけ点群密度を変更するなど，柔軟なpathを生成することができ，課題として与えられた図形の正確な再現を保証します。

### 2️⃣ **シミュレーションプログラム**
- **概要**: 描画条件を入力として、ロボットの動作(軌道)を仮想平面でシミュレーションします。
- **入力変数例**:
  - 軌道座標列
  - 第一，第二リンクアームのそれぞれの長さ
  - PID制御のゲイン
  - 制御周期
  - 軌道座標更新周期
  - 描画図形の初期位置
- **特徴**: 設定条件に基づく動作の可視化を可能にし、最適な制御パラメータを見つけ，アームの設計に適用するための補助ツールです。
  このプログラムを使うことで，第一，第二リンクの腕の長さや，制御周期などの変数を，直感的に把握しやすくなります．

### 3️⃣ **実機制御プログラム**
- **概要**: Arduino上で動作する制御プログラムです。モーターを操作し、実際の描画タスクを実行します。
- **特徴**: 逆運動学計算アルゴリズムにより，与えられた座標配列から，正確に図形をなぞるためのモーターの回転角度を割り出し，その角度に対してPID制御で追従します．
  - コンフィグファイルに軌道を書き込み，そのファイルから目標軌道座標配列を読み取りながら制御に使用します．

---

### 🔍 **注目ポイント**
- **aruduino IDEの使用**:  matlabではなく，aruduinoで開発
- **柔軟性**: 異なるハードウェア構成や制御パラメータを簡単に変更し，挙動を試すことができます．容易にトライアンドエラーができるプラットフォームを目指しました．
- **効率性**: 経路生成から実機描画までのプロセスをシームレスに統合しています．

---

## 🛠 **使い方**
以下の手順でシステムをセットアップし、動作を確認できます。
1. リポジトリをクローンします:
    ```bash
    git clone https://github.com/LaLaSero/one-stroke.git
    cd one-stroke
    ```
2. 必要なライブラリをインストールしてください

### 3️⃣ パス生成
- 図形データから描画用パスを生成します。
    ```bash
    cd path
    python3 arc_square_path.py
    sh format.sh arc_square.csv
    ```
     x_arc_square.txtとy_arc_square.txtが生成されるので，それらを simulator/arc/ に移動させます

### 4️⃣ シミュレーション
- シミュレーションプログラムで生成したパスをテストします。
    ```bash
    cd ../simulator
    python3 sim_arc.py
    ```
### 5 変更の適用
-  ```bash
   cd ../one-stroke
   
   ```

この構成を活かし、ロボット描画タスクを完璧にこなすプラットフォームを目指しました。

