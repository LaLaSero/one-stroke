# プロジェクト名

![ライセンス](https://img.shields.io/github/license/ユーザー名/リポジトリ名)
![言語](https://img.shields.io/github/languages/top/ユーザー名/リポジトリ名)
![リリース](https://img.shields.io/github/v/release/ユーザー名/リポジトリ名)

# ロボット描画システム：一筆書きで描く高速・高精度ロボット

## 🎯 **プロジェクトの目的**
このプロジェクトでは、課題として提示された図形を **速く**、**正確に**、そして **一筆書きで描く** ことを目的としたロボットを開発しました。

---

## 🛠 **ハードウェア構成**
本ロボットの構成は以下の通りです：

- **制御マイコン**: Arduino Mega 2560（互換品）
- **駆動部**: モーターユニット × 2
- **リンク機構**: 描画アーム（リンク付き）

---

## 🧠 **制御プログラム**
ロボットを制御するためのプログラムは、以下の3つの主要モジュールで構成されています：

### 1️⃣ **パス生成プログラム**
- **概要**: 与えられた図形データをもとに、描画用の経路（path）を計算・生成します。
- **特徴**: 精密なパス生成アルゴリズムを採用し、図形の正確な再現を保証します。

### 2️⃣ **シミュレーションプログラム**
- **概要**: 描画条件を入力として、ロボットの動作(軌道)を仮想平面でシミュレーションします。
- **入力項目例**:
  - リンクアームの長さ
  - PID制御のゲイン
  - 描画図形の初期位置
- **特徴**: 設定条件に基づく動作の可視化を可能にし、最適な制御パラメータを見つけるためのツールです。

### 3️⃣ **実機制御プログラム**
- **概要**: Arduino上で動作する制御プログラムです。モーターを操作し、実際の描画タスクを実行します。
- **特徴**: パス生成プログラムの出力を直接取り込み、リアルタイム制御を実現します。

---

### 🔍 **注目ポイント**
- **柔軟性**: ハードウェア構成や制御パラメータを簡単に変更可能。
- **効率性**: 経路生成から実機描画までのプロセスをシームレスに統合。
- **再現性**: 高精度の一筆書き描画で一貫性を確保。

---

## 🛠 **使い方**
以下の手順でシステムをセットアップし、動作を確認できます。
1. リポジトリをクローンします:
    ```bash
    git clone https://github.com/ユーザー名/リポジトリ名.git
    cd リポジトリ名
    ```
2. 必要なライブラリをインストールしてください（例: Arduino IDEのライブラリマネージャを使用）。

### 3️⃣ パス生成
- 図形データを `path_generator` プログラムに入力し、描画用パスを生成します。
    ```bash
    python path_generator.py --input example_shape.json --output path.json
    ```

### 4️⃣ シミュレーション
- シミュレーションプログラムで生成したパスをテストします。
    ```bash
    python simulation.py --path path.json
    ```

この構成を活かし、ロボット描画タスクを完璧にこなすプラットフォームを目指しました。

