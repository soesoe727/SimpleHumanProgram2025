# SpaceMouse Integration for SimpleHumanGLUT

このドキュメントは、SpaceMouseを使用してSimpleHumanGLUTで立方体を回転・移動させるための統合モジュールの使用方法を説明します。

## ファイル構成

### コアライブラリ（再利用可能）

1. **CSpaceMouseTransform.hpp/cpp**
   - vecmathのmatrix4f型で座標変換行列を管理
   - スレッドセーフな行列操作
   - 平行移動と回転の適用
   - プロジェクト固有のヘッダへの依存なし

2. **CSpaceMouseController.hpp/cpp**
   - SpaceMouse SDKとの統合レイヤー
   - CSpaceMouseTransformの更新を管理
   - 感度調整機能

3. **SpaceMouseGLUTHelper.hpp/cpp**
   - GLUT アプリケーション用のシンプルなインターフェース
   - シングルトンパターンで簡単に使用可能
   - OpenGLマトリックスへの直接適用

### サンプル/ドキュメント

4. **SpaceMouseExample.cpp**
   - 統合方法のサンプルコード
   - キーボードシミュレーション機能
   - 詳細なコメント付き

## 使用方法

### 1. 基本的な初期化（SimpleHumanGLUT.cppで）

```cpp
#include "SpaceMouseGLUTHelper.hpp"

// main関数または初期化関数で
int main(int argc, char** argv) {
    // GLUTの初期化...
    
    // SpaceMouseの初期化
    using namespace SpaceMouseHelper;
    CSpaceMouseGLUT::GetInstance().Initialize("SimpleHumanGLUT Demo");
    CSpaceMouseGLUT::GetInstance().SetSensitivity(0.01f);
    
    // ...
}
```

### 2. Display関数での使用

```cpp
void GLUTBaseApp::Display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // カメラ行列の設定
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -camera_distance);
    glRotatef(-camera_pitch, 1.0, 0.0, 0.0);
    glRotatef(-camera_yaw, 0.0, 1.0, 0.0);
    glTranslatef(-view_center.x, -view_center.y, -view_center.z);
    
    // SpaceMouseによる変換を適用
    SpaceMouseHelper::CSpaceMouseGLUT::GetInstance().ApplyToGLMatrix();
    
    // オブジェクトを描画
    DrawCube();
    // ...
}
```

### 3. キーボードハンドラーでのリセット機能（オプション）

```cpp
void GLUTBaseApp::Keyboard(unsigned char key, int x, int y) {
    if (key == 'r' || key == 'R') {
        // 変換行列をリセット
        SpaceMouseHelper::CSpaceMouseGLUT::GetInstance().Reset();
        cout << "SpaceMouse transform reset" << endl;
    }
    // ... 他のキー処理
}
```

### 4. クリーンアップ

```cpp
// アプリケーション終了時
SpaceMouseHelper::CSpaceMouseGLUT::GetInstance().Shutdown();
```

## 高度な使用方法

### 行列を直接取得する

OpenGLに直接適用せず、独自の処理で行列を使いたい場合：

```cpp
Matrix4f transform;
SpaceMouseHelper::CSpaceMouseGLUT::GetInstance().GetTransformMatrix(transform);

// transformを使用して頂点を変換するなど
Point3f vertex(1.0f, 0.0f, 0.0f);
Point3f transformedVertex;
transform.transform(vertex, transformedVertex);
```

### 手動での変換適用（テスト用）

SpaceMouseデバイスがない場合、キーボードで動作をシミュレート：

```cpp
// 平行移動
CSpaceMouseGLUT::GetInstance().ApplyTranslation(0.1f, 0.0f, 0.0f);

// 回転（ラジアン、軸）
CSpaceMouseGLUT::GetInstance().ApplyRotation(0.1f, 0.0f, 1.0f, 0.0f);

// ローカル座標系での回転
CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(0.1f, 0.0f, 1.0f, 0.0f);

// SpaceMouseモーション入力のシミュレーション（推奨）
// このメソッドはApplyLocalMotionを使用し、最も自然な操作感を提供します
CSpaceMouseGLUT::GetInstance().ApplyMotionInput(
    0.05f, 0.02f, 0.0f,  // 平行移動 (tx, ty, tz)
    0.05f, 0.03f, 0.0f   // 回転 (rx, ry, rz) ラジアン
);
```

## プロジェクトへの追加

### Visual Studioプロジェクト設定

1. 以下のファイルをプロジェクトに追加：
   - CSpaceMouseTransform.hpp/cpp
   - CSpaceMouseController.hpp/cpp
   - SpaceMouseGLUTHelper.hpp/cpp

2. インクルードディレクトリに以下を追加：
   - vecmathのパス
   - SpaceMouse SDKのincludeパス

3. リンカー設定：
   - 必要なSpaceMouse SDKライブラリをリンク

## 設計の特徴

### ApplyLocalMotionの使用
- 実装された`ApplyMotionInput`メソッドは内部で`ApplyLocalMotion`を使用します
- これにより、回転がオブジェクトのローカル座標系で適用され、直感的な操作感が得られます
- ワールド座標系での回転（`ApplyMotion`）と比較して、オブジェクト自身の軸周りの回転が自然に行えます

### 依存関係の最小化
- **CSpaceMouseTransform**: vecmathのみに依存（完全に再利用可能）
- **CSpaceMouseController**: 最小限のSDK依存
- **SpaceMouseGLUTHelper**: GLUTアプリケーション向けの薄いラッパー

### スレッドセーフ
- CSpaceMouseTransformは内部でmutexを使用し、複数スレッドからのアクセスに対応

### シングルトンパターン
- SpaceMouseGLUTHelperはシングルトンで、グローバルアクセスが容易

## トラブルシューティング

### SpaceMouseが反応しない場合
1. デバイスが正しく接続されているか確認
2. 3Dconnexionドライバーがインストールされているか確認
3. 感度設定を調整：`SetSensitivity(1.0f)` など

### コンパイルエラー
1. vecmathのパスが正しく設定されているか確認
2. SpaceMouse SDKのヘッダファイルが見つかるか確認

## 参考情報

- vecmath C++ライブラリ: http://objectclub.jp/download/vecmath1
- 3Dconnexion SDK ドキュメント
- SimpleHumanGLUT オリジナルプログラム

## ライセンス

このコードは、3Dconnexion SDKのライセンスに従います。
詳細は"LicenseAgreementSDK.txt"を参照してください。
