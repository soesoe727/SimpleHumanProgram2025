/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  姿勢補間アプリケーション
**/

#ifndef  _POSTURE_INTERPOLATION_APP_H_
#define  _POSTURE_INTERPOLATION_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"


//
//  姿勢補間アプリケーションクラス
//
class  PostureInterpolationApp : public GLUTBaseApp
{
  protected:
	// 姿勢補間の入力情報

	// サンプル姿勢
	Posture *  posture0;
	Posture *  posture1;

	// 姿勢補間の重み
	float  weight;

  protected:
	// 姿勢補間のための変数

	// キャラクタの姿勢
	Posture *  curr_posture;

	// 現在姿勢の描画色
	Color3f  figure_color;

  protected:
	// 描画設定

	// サンプル姿勢の描画色
	Color3f  posture0_color;
	Color3f  posture1_color;

	// 補間姿勢の腰の位置を固定して描画
	bool  draw_fixed_position;


  public:
	// コンストラクタ
	PostureInterpolationApp();

	// デストラクタ
	virtual ~PostureInterpolationApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

  public:
	// 補助処理

	// 姿勢更新
	void  UpdatePosture();
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// 姿勢補間（２つの姿勢を補間）（※レポート課題）
void  MyPostureInterpolation( const Posture & p0, const Posture & p1, float ratio, Posture & p );


#endif // _POSTURE_INTERPOLATION_APP_H_
