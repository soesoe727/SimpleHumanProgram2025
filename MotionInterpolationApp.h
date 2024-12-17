/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作補間アプリケーション
**/

#ifndef  _MOTION_INTERPOLATION_APP_H_
#define  _MOTION_INTERPOLATION_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"

// プロトタイプ宣言
struct  MotionInfo;


//
//   動作補間アプリケーションクラス
//
class  MotionInterpolationApp : public GLUTBaseApp
{
  protected:
	// 動作補間の入力情報

	// 動作データ情報（動作補間に用いる２つの動作）
	MotionInfo * motions[ 2 ];

	// 動作補間の重み
	float  weight;

  protected:
	// 動作再生のための変数

	// キャラクタの姿勢
	Posture *  curr_posture;

	// アニメーション中かどうかを表すフラグ
	bool  on_animation;

	// アニメーションの再生時間
	float  animation_time;

	// アニメーションの再生速度
	float  animation_speed;

  protected:
	// 動作補間のための変数

	// サンプル動作からの取得姿勢
	Posture *  motion_posture[ 2 ];

	// 現在姿勢の描画色
	Color3f  figure_color;

  protected:
	// 動作接続のための変数

	// 繰り返し動作の再生開始時刻
	float  cycle_start_time;

	// サンプル動作の動作接続（前の動作の終了時の位置・向きに合わせる）ための変換行列
	Matrix4f  motion_trans_mat[ 2 ];


  public:
	// コンストラクタ
	MotionInterpolationApp();

	// デストラクタ
	virtual ~MotionInterpolationApp();

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

	// アニメーション処理
	virtual void  Animation( float delta );

  public:
	// 補助処理

	// 動作再生処理（動作補間＋動作接続）
	void  AnimationWithInterpolation( float delta );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// ２つの姿勢の位置・向きを合わせるための変換行列を計算
//（next_frame の位置・向きを、prev_frame の位置向きに合わせるための変換行列 trans_mat を計算）
void  ComputeConnectionTransformation( const Matrix4f & prev_frame, const Matrix4f & next_frame, Matrix4f & trans_mat );

// 姿勢の位置・向きに変換行列を適用
void  TransformPosture( const Matrix4f & trans, Posture & posture );


#endif // _MOTION_INTERPOLATION_APP_H_
