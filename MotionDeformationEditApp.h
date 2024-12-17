/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作変形アプリケーション
**/

#ifndef  _MOTION_DEFORMATION_EDIT_APP_H_
#define  _MOTION_DEFORMATION_EDIT_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"

#include "MotionDeformationApp.h"


//
//  動作変形（＋編集機能）アプリケーションクラス
//
class  MotionDeformationEditApp : public  MotionDeformationApp
{
  protected:
	// アプリケーションの状態

	// 変形動作再生モードかを表すフラグ
	bool               on_animation_mode;

	// キー姿勢編集モードかを表すフラグ
	bool               on_keypose_edit_mode;

	// 逆運動学計算によるキー姿勢の編集には基底クラス（InverseKinematicsCCDApp）の変数・メソッドを使用


  public:
	// コンストラクタ
	MotionDeformationEditApp();

	// デストラクタ
	virtual ~MotionDeformationEditApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	//  マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );

  protected:
	// 内部処理

	// 変形動作再生モードの開始
	void  StartAnimationMode();

	// キー姿勢編集モードの開始
	void  StartKeyposeEditMode();

	// キー姿勢の初期化
	void  ResetKeypose();
};


#endif // _MOTION_DEFORMATION_APP_H_
