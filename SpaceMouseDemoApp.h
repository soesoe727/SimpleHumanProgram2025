#pragma once


// ライブラリ・クラス定義の読み込み
#include "SimpleHumanGLUT.h"
#include <Matrix4.h>


//
//  SpaceMouse デモアプリケーションクラス
//
class  SpaceMouseDemoApp : public GLUTBaseApp
{
  public:
	// コンストラクタ
	SpaceMouseDemoApp();

	// デストラクタ
	virtual ~SpaceMouseDemoApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );

  private:
	// 直方体を描画
	void  DrawBox();

	// 直方体の座標変換行列
	Matrix4f m_boxTransform;
};
