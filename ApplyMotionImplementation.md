# ApplyMotion実装ドキュメント

## 概要

このドキュメントでは、SpaceMouseの3D入力を処理するために実装された`ApplyMotionInput`メソッドについて説明します。

## 実装されたメソッド

### CSpaceMouseController::ApplyMotionInput

```cpp
void CSpaceMouseController::ApplyMotionInput(
    float tx, float ty, float tz,  // 平行移動
    float rx, float ry, float rz   // 回転（ラジアン）
);
```

このメソッドは、SpaceMouseからのモーション入力を処理し、内部で`CSpaceMouseTransform::ApplyLocalMotion`を呼び出します。

## 使用される座標系

### ApplyLocalMotion（実装済み）

- **座標系**: ローカル（オブジェクト固有の座標系）
- **回転の適用**: `m_transform = m_transform * rotation`
- **特徴**: オブジェクト自身の軸周りに回転するため、直感的な操作感

### ApplyMotion（実装済みだが未使用）

- **座標系**: ワールド（グローバル座標系）
- **回転の適用**: `m_transform = rotation * m_transform`
- **特徴**: ワールド座標の固定軸周りに回転

## なぜApplyLocalMotionを使用するのか

1. **直感的な操作**
   - オブジェクトがどの方向を向いていても、SpaceMouseの操作軸はオブジェクトの現在の向きに従います
   - 例: オブジェクトが回転していても、「上方向」への入力は常にオブジェクトのローカルY軸方向

2. **3D CADソフトウェアとの一貫性**
   - Blender、Maya、SolidWorksなど主要な3Dソフトウェアは、ローカル座標系での回転をデフォルトとしています

3. **累積回転の自然さ**
   - 連続的な回転操作が自然に振る舞います
   - ジンバルロック問題を回避しやすい

## コード例

### 基本的な使用方法

```cpp
// SpaceMouseデバイスからのイベントハンドラ内で
void OnSpaceMouseMotion(float tx, float ty, float tz, 
                        float rx, float ry, float rz) {
    CSpaceMouseGLUT::GetInstance().ApplyMotionInput(
        tx, ty, tz,  // 平行移動成分
        rx, ry, rz   // 回転成分（ラジアン）
    );
}
```

### キーボードシミュレーション

```cpp
// 'M'キーで複合的な動きをシミュレート
case 'M':
{
    float tx = 0.05f;  // X方向に移動
    float ty = 0.02f;  // Y方向に移動
    float tz = 0.0f;
    float rx = 0.05f;  // X軸周りに回転
    float ry = 0.03f;  // Y軸周りに回転
    float rz = 0.0f;
    
    CSpaceMouseGLUT::GetInstance().ApplyMotionInput(tx, ty, tz, rx, ry, rz);
}
break;
```

## 実装の詳細

### CSpaceMouseTransform::ApplyLocalMotion

```cpp
void CSpaceMouseTransform::ApplyLocalMotion(
    const Vector3f &translation, 
    const Vector3f &rotation, 
    float scale) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // 平行移動を適用
    if (translation.lengthSquared() > 1e-6f) {
        Matrix4f trans = CreateTranslationMatrix(
            translation.x * scale,
            translation.y * scale,
            translation.z * scale
        );
        m_transform = trans * m_transform;
    }
    
    // 回転を適用（ローカル座標系）
    float rotation_magnitude = rotation.length();
    if (rotation_magnitude > 1e-6f) {
        float angle = rotation_magnitude * scale;
        Vector3f axis = rotation;
        axis.normalize();
        
        Matrix4f rot = CreateRotationMatrix(angle, axis.x, axis.y, axis.z);
        m_transform = m_transform * rot;  // 右乗算でローカル回転
    }
}
```

### 重要な点

1. **右乗算**: `m_transform = m_transform * rot` によりローカル座標系での回転を実現
2. **スレッドセーフ**: mutexによる保護で複数スレッドから安全に呼び出し可能
3. **スケーリング**: `m_motionScale`により感度を調整可能

## 感度の調整

```cpp
// 初期化時に感度を設定
CSpaceMouseGLUT::GetInstance().SetSensitivity(0.01f);

// 低い値 (0.001-0.01): 精密な操作に適している
// 中程度の値 (0.01-0.1): 通常の操作に適している  
// 高い値 (0.1-1.0): 素早い大きな動きに適している
```

## まとめ

`ApplyMotionInput`メソッドは、`ApplyLocalMotion`を使用することで、SpaceMouseの入力を直感的かつ自然にオブジェクトの変換に反映します。この実装は、業界標準の3Dアプリケーションと同様の操作感を提供します。
