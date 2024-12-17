/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作接続・遷移アプリケーション
**/

#ifndef  _MOTION_TRANSITION_APP_H_
#define  _MOTION_TRANSITION_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"


// プロトタイプ宣言
struct  MotionInfo;
class  MotionTransition;
class  Timeline;


//
//  動作接続・遷移アプリケーションクラス
//
class  MotionTransitionApp : public GLUTBaseApp
{
  protected:
	// 動作遷移・接続（を含む動作再生）の入力情報

	// 動作データリスト
	vector< MotionInfo * >  motion_list;

	// 現在の再生動作番号
	int  curr_motion_no;

	// 現在の動作の位置・向きに対する変換行列
	Matrix4f  curr_motion_mat;

	// 現在の動作の再生開始時刻（グローバル時刻）
	float  curr_start_time;

	// 次の再生動作番号
	int  next_motion_no;

	// 実行待ちの動作番号
	int  waiting_motion_no;

  protected:
	// 動作遷移のための変数

	// 動作接続・遷移
	MotionTransition *  transition;

	// 動作遷移（前後の動作のブレンディング）を適用するかどうかの設定
	bool  enable_transition;

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
	// 情報表示用の変数

	// 現在姿勢の描画色
	Color3f  figure_color;

	// タイムライン描画機能
	Timeline *  timeline;

	// 動作遷移回数のカウント（タイムライン表示用）
	int  transition_count;


  public:
	// コンストラクタ
	MotionTransitionApp();

	// デストラクタ
	virtual ~MotionTransitionApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// ウィンドウサイズ変更
	virtual void  Reshape( int w, int h );

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// キーボードの特殊キー押下
	virtual void  KeyboardSpecial( unsigned char key, int mx, int my );
	
	// アニメーション処理
	virtual void  Animation( float delta );

  public:
	// 補助処理

	//  次の動作を変更
	void  SetNextMotion( int no = -1 );

	// 動作再生処理（動作接続・遷移を考慮）
	void  AnimationWithMotionTransition( float delta );

	// タイムラインへの前後の動作・遷移区間の設定
	static void  InitTimeline( Timeline & timeline, const MotionTransition & trans, int count );

	// タイムラインへの現在時刻・時間範囲の設定
	static void  UpdateTimeline( Timeline & timeline, float curr_time );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// サンプル動作セットの読み込み
const Skeleton *  LoadSampleMotions( vector< MotionInfo * > &  motion_list, const Skeleton * body = NULL );


#endif // _MOTION_TRANSITION_APP_H_
