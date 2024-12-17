/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作補間アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "MotionInterpolationApp.h"
#include "MotionTransition.h"
#include "MotionTransitionApp.h"
#include "PostureInterpolationApp.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  コンストラクタ
//
MotionInterpolationApp::MotionInterpolationApp()
{
	app_name = "Motion Interpolation";

	motions[ 0 ] = NULL;
	motions[ 1 ] = NULL;
	weight = 0.0f;

	curr_posture = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;

	motion_posture[ 0 ] = NULL;
	motion_posture[ 1 ] = NULL;
	figure_color.set( 1.0f, 1.0f, 1.0f );

	cycle_start_time = 0.0f;
	motion_trans_mat[ 0 ].rotY( M_PI );
	motion_trans_mat[ 1 ].rotY( M_PI );
}


//
//  デストラクタ
//
MotionInterpolationApp::~MotionInterpolationApp()
{
	for ( int i=0; i<2; i++ )
	{
		if ( motions[ i ] )
		{
			if ( motions[ i ]->motion )
				delete  motions[ i ]->motion;
			delete  motions[ i ];
		}
		if ( motion_posture[ i ])
			delete  motion_posture[ i ];
	}

	if ( curr_posture && curr_posture->body )
		delete  curr_posture->body;
	if ( curr_posture )
		delete  curr_posture;
}


//
//  初期化
//
void  MotionInterpolationApp::Initialize()
{
	GLUTBaseApp::Initialize();

	const Skeleton *  body = NULL;

	// サンプル動作セットの読み込み
	if ( !motions[ 0 ] )
	{
		vector< MotionInfo * >  motion_list;
		body = LoadSampleMotions( motion_list );
		motions[ 0 ] = motion_list[ 0 ];
		motions[ 1 ] = motion_list[ 1 ];
	}

	// 姿勢の初期化
	if ( body )
	{
		if ( curr_posture )
			delete  curr_posture;
		if ( motion_posture[ 0 ] )
			delete  motion_posture[ 0 ];
		if ( motion_posture[ 1 ] )
			delete  motion_posture[ 1 ];

		curr_posture = new Posture( body );
		InitPosture( *curr_posture, body );
		motion_posture[ 0 ] = new Posture( body );
		motion_posture[ 1 ] = new Posture( body );
	}
}


//
//  開始・リセット
//
void  MotionInterpolationApp::Start()
{
	GLUTBaseApp::Start();

	weight = 0.0f;

	on_animation = true;
	animation_time = 0.0f;

	cycle_start_time = 0.0f;

	InitPosture( *curr_posture );

	// 各サンプル動作の接続のための変換行列を計算
	Matrix4f  init_frame;
	init_frame.setIdentity();
	for ( int i=0; i<2; i++ )
	{
		// 動作データが読み込まれていなければスキップ
		if ( !motions[ i ] || !motions[ i ]->motion || !curr_posture )
			continue;

		// 動作データから開始姿勢を取得、位置・向きを取得
		motions[ i ]->motion->GetPosture( motions[ i ]->keytimes[ 0 ], *motion_posture[ i ] );

		// 現在の動作の終了姿勢と次の動作の開始姿勢の姿勢の位置・向きを合わせるための変換行列を計算
		//（curr_posture の位置・向きを、next_motion_posture の位置向きに合わせるための変換行列 trans_mat を計算）
		ComputeConnectionTransformation( *curr_posture, 0.0f, *motion_posture[ i ], 180.0f, motions[ i ]->base_segment_no, motion_trans_mat[ i ] );
	}
	
	Animation( 0.0f );
}


//
//  画面描画
//
void  MotionInterpolationApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( figure_color.x, figure_color.y, figure_color.z );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// 現在のモード、補間重みを表示
	DrawTextInformation( 0, "Motion Interpolation" );
	char  message[ 64 ];
	sprintf( message, "%.2f", animation_time );
	DrawTextInformation( 1, message );
	sprintf( message, "Weight: %.2f", weight );
	DrawTextInformation( 2, message );
}


//
//  マウスドラッグ
//
void  MotionInterpolationApp::MouseDrag( int mx, int my )
{
	// 左ボタンの左右ドラッグに応じて重みを計算
	if ( drag_mouse_l )
	{
		// 重み計算
		weight += (float) ( mx - last_mouse_x ) * 2.0f / win_width;
		if ( weight < 0.0f )
			weight = 0.0f;
		if ( weight > 1.0f )
			weight = 1.0f;

		// 姿勢更新
		if ( on_animation )
			Animation( 0.0f );
		else
		{
			on_animation = true;
			Animation( 0.0f );
			on_animation = false;
		}
	}

	GLUTBaseApp::MouseDrag( mx, my );
}


//
//  キーボードのキー押下
//
void  MotionInterpolationApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// s キーでアニメーションの停止・再開
	if ( key == 's' )
		on_animation = !on_animation;

	// w キーでアニメーションの再生速度を変更
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

	// n キーで次のフレーム
	if ( ( key == 'n' ) && !on_animation && motions[ 0 ] )
	{
		on_animation = true;
		Animation( motions[ 0 ]->motion->interval );
		on_animation = false;
	}

	// p キーで前のフレーム
	if ( ( key == 'p' ) && !on_animation && motions[ 0 ] )
	{
		on_animation = true;
		Animation( - motions[ 0 ]->motion->interval );
		on_animation = false;
	}
}


//
//  アニメーション処理
//
void  MotionInterpolationApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// 動作再生処理（動作補間＋動作接続）
	AnimationWithInterpolation( delta );

	// 注視点を更新
	view_center.set( curr_posture->root_pos.x, 0.0f, curr_posture->root_pos.z );
}


//
//  動作再生処理（動作補間＋動作接続）
//
void  MotionInterpolationApp::AnimationWithInterpolation( float delta )
{
	// 補間動作のキー時刻の配列
	int  num_keyframes = motions[ 0 ]->keytimes.size();
	vector< float >  keytimes( num_keyframes );

	// 補間動作の現在時刻（動作開始時を基準とする時間）
	float  local_time = 0.0f;

	// 正規化時間（区間番号と区間内の正規化時間）を計算
	int  seg_no = 0;
	float  seg_time = 0.0f;

	// 各サンプル動作の現在時刻（動作データ内の時間）
	float  motion_time = 0.0f;


	// 時間を進める
	animation_time += delta * animation_speed;

	// サンプル動作が設定されていなければ終了
	if ( !motions[ 0 ] || !motions[ 1 ] || !motions[ 0 ]->motion || !motions[ 1 ]->motion )
		return;

	// 補間動作のキー時刻を計算（サンプル動作のキー時刻を重みで平均）
	keytimes[ 0 ] = 0.0f;
	for ( int i = 1; i < num_keyframes; i++ )
	{
		// ※ レポート課題
//		keytimes[ i ] = ???;
	}

	// 補間動作の現在時刻（動作開始時を基準とする時間）を計算
	local_time = animation_time - cycle_start_time;

	// 現在の繰り返し動作が終了したら、次の繰り返しを開始
	if ( local_time > keytimes[ num_keyframes - 1 ] )
	{
		cycle_start_time += keytimes[ num_keyframes - 1 ];
		local_time = animation_time - cycle_start_time;

		// 各サンプル動作の接続のための変換行列を計算
		for ( int i = 0; i < 2; i++ )
		{
			// 動作データから開始姿勢を取得、位置・向きを取得
			motions[ i ]->motion->GetPosture( motions[ i ]->keytimes[ 0 ], *motion_posture[ i ] );

			// 現在の動作の終了姿勢と次の動作の開始姿勢の姿勢の位置・向きを合わせるための変換行列を計算
			//（curr_posture の位置・向きを、next_motion_posture の位置向きに合わせるための変換行列 trans_mat を計算）
			ComputeConnectionTransformation( *curr_posture, 0.0f, *motion_posture[ i ], 180.0f, motions[ i ]->base_segment_no, motion_trans_mat[ i ] );
		}
	}

	// 正規化時間（区間番号と区間内の正規化時間）を計算
	for ( int i = 0; i < num_keyframes-1; i++ )
	{
		if ( ( local_time >= keytimes[ i ] ) && ( local_time <= keytimes[ i + 1 ] ) )
		{
			seg_no = i;

			// ※ レポート課題
//			seg_time = ???;

			break;
		}
	}

	// サンプル動作から姿勢を取得
	for ( int i = 0; i < 2; i++ )
	{
		// 各サンプル動作の現在時刻（動作データ内の時間）を計算

		// ※ レポート課題
//		motion_time = ???;

		// 動作データから姿勢を取得
		motions[ i ]->motion->GetPosture( motion_time, *motion_posture[ i ] );

		// 前の動作の終了時の位置・向きに合わせるための変換行列を適用
		TransformPosture( motion_trans_mat[ i ], *motion_posture[ i ] );
	}

	MyPostureInterpolation( *motion_posture[ 0 ], *motion_posture[ 1 ], weight, *curr_posture );

	// 重みに応じて描画色を設定
	figure_color.scaleAdd( weight, motions[ 1 ]->color - motions[ 0 ]->color, motions[ 0 ]->color );
}


