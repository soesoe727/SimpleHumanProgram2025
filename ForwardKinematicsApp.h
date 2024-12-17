/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  順運動学計算アプリケーション
**/

#ifndef  _FORWARD_KINEMATICS_APP_H_
#define  _FORWARD_KINEMATICS_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "MotionPlaybackApp.h"


//
//  順運動学計算アプリケーションクラス
// （動作再生アプリケーションに順運動学計算を追加）
//
class  ForwardKinematicsApp : public MotionPlaybackApp
{
  protected:
	// 順運動学計算結果の変数

	// 全体節の位置・向き
	vector< Matrix4f >  segment_frames;
	
	// 全関節の位置
	vector< Point3f >  joint_positions;

  public:
	// コンストラクタ
	ForwardKinematicsApp();

  public:
	// イベント処理

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// アニメーション処理
	virtual void  Animation( float delta );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// 順運動学計算（※レポート課題）
void  MyForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );

// 順運動学計算のための反復計算（ルート体節から末端体節に向かって繰り返し再帰呼び出し）（※レポート課題）
void  MyForwardKinematicsIteration( const Segment *  segment, const Segment * prev_segment, const Posture & posture, 
	Matrix4f * seg_frame_array, Point3f * joi_pos_array = NULL );


#endif // _FORWARD_KINEMATICS_APP_H_
