/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  キーフレーム動作再生アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "KeyframeMotionPlaybackApp.h"

// プロトタイプ宣言

// 姿勢補間（２つの姿勢を補間）（※レポート課題）
void  MyPostureInterpolation( const Posture & p0, const Posture & p1, float ratio, Posture & p );



//
//  コンストラクタ
//
KeyframeMotionPlaybackApp::KeyframeMotionPlaybackApp()
{
	app_name = "Keyframe Motion Playback";

	keyframe_motion = NULL;
	keyframe_posture = NULL;
	motion_time_offset = 0.0f;

	draw_original_motion = true;
	draw_key_poses = true;
}


//
//  デストラクタ
//
KeyframeMotionPlaybackApp::~KeyframeMotionPlaybackApp()
{
	if ( keyframe_motion )
		delete  keyframe_motion;
	if ( keyframe_posture )
		delete  keyframe_posture;
}


//
//  初期化
//
void  KeyframeMotionPlaybackApp::Initialize()
{
	GLUTBaseApp::Initialize();

	const char *  sample_motions = "sample_walking1.bvh";
	const int  num_keytimes = 3;
	const float  sample_keytimes[ num_keytimes ] = { 2.35f, 3.00f, 3.74f };

	// BVH動作データを読み込み
	LoadBVH( sample_motions );
	if ( !motion )
		return;

	// キーフレームの時刻を設定
	vector< float >  key_times( num_keytimes );
	for ( int i = 0; i < num_keytimes; i++ )
		key_times[ i ] = sample_keytimes[ i ] - sample_keytimes[ 0 ];

	// BVH動作から取得した姿勢をキーフレームの姿勢として設定
	vector< Posture >  key_poses( num_keytimes );
	for ( int i = 0; i < num_keytimes; i++ )
		motion->GetPosture( sample_keytimes[ i ], key_poses[ i ] );

	// キーフレーム動作を初期化
	keyframe_motion = new KeyframeMotion();
	keyframe_motion->Init( motion->body, num_keytimes, &key_times.front(), &key_poses.front() );

	// キーフレーム動作から取得する姿勢の初期化
	keyframe_posture = new Posture( motion->body );
	*keyframe_posture = key_poses[ 0 ];

	// キーフレーム動作と元の動作を同期して再生するための時間のオフセットを設定
	motion_time_offset = sample_keytimes[ 0 ];

	// サンプルBVH動作に合わせて視点調節
	camera_yaw += 180.0f;
}


//
//  画面描画
//
void  KeyframeMotionPlaybackApp::Display()
{
	GLUTBaseApp::Display();

	// キーフレーム動作から取得した姿勢を描画
	if ( keyframe_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *keyframe_posture );
		DrawPostureShadow( *keyframe_posture, shadow_dir, shadow_color );
	}

	// 元のBVH動作から取得した姿勢を描画
	if ( draw_original_motion && curr_posture )
	{
		glPushMatrix();
		glTranslatef( 1.0f, 0.0f, 0.0f );

		glColor3f( 0.0f, 0.0f, 1.0f );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
		glPopMatrix();
	}

	// キーフレーム動作のキー姿勢を描画
	if ( draw_key_poses && keyframe_posture )
	{
		glPushMatrix();
		glTranslatef( - 1.0f, 0.0f, 0.0f );

		for ( int i=0; i<keyframe_motion->num_keyframes; i++ )
		{
			glPushMatrix();
			glTranslatef( 0.0f, 0.0f, 1.0f * ( i - ( keyframe_motion->num_keyframes - 1 ) * 0.5f ) );

			glColor3f( 0.8f, 0.8f, 0.8f );
			DrawPosture( keyframe_motion->key_poses[ i ] );
			DrawPostureShadow( keyframe_motion->key_poses[ i ], shadow_dir, shadow_color );
			glPopMatrix();
		}
		glPopMatrix();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Keyframe Motion Playback" );
	char  message[64];
	if ( motion )
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
	else
		sprintf( message, "Press 'L' key to Load a BVH file" );
	DrawTextInformation( 1, message );
}


//
//  キーボードのキー押下
//
void  KeyframeMotionPlaybackApp::Keyboard( unsigned char key, int mx, int my )
{
	// 基底クラスの処理を実行
	MotionPlaybackApp::Keyboard( key, mx, my );

	// d キーで描画設定を変更
	if ( key == 'd' )
	{
		if ( draw_original_motion && draw_key_poses )
			draw_key_poses = false;
		else if ( draw_original_motion && !draw_key_poses )
		{
			draw_original_motion = false;
			draw_key_poses = true;
		}
		else if ( !draw_original_motion && draw_key_poses )
			draw_key_poses = false;
		else
		{
			draw_original_motion = true;
			draw_key_poses = true;
		}
	}
}


//
//  アニメーション処理
//
void  KeyframeMotionPlaybackApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// 動作データが読み込まれていなければ終了
	if ( !keyframe_motion )
		return;

	// 時間を進める
	animation_time += delta * animation_speed;;
	if ( animation_time > keyframe_motion->GetDuration() )
		animation_time -= keyframe_motion->GetDuration();
	frame_no = ( animation_time + motion_time_offset ) / motion->interval;

	// 動作データから現在時刻の姿勢を取得
	motion->GetPosture( animation_time + motion_time_offset, *curr_posture );

	// キーフレーム動作データから現在時刻の姿勢を取得
	GetKeyframeMotionPosture( *keyframe_motion, animation_time, *keyframe_posture );
}


//
//  キーフレーム動作から姿勢を取得
//
void  GetKeyframeMotionPosture( const KeyframeMotion & motion, float time, Posture & p )
{
	// ※レポート課題

	// 指定時刻に対応する区間の番号を取得
	int  no = -1;
//	for ( int i = 0; i < ???; i++ )
	{
//		if ( ??? )
		{
//			no = i;
//			break;
		}
	}

	// 対応する区間が存在しない場合は終了
	if ( no == -1 )
		return;

	// 補間の割合を計算
	float  s = 0.0f;
//	s = ??? / ???;

	// 前後のキー姿勢を補間
	MyPostureInterpolation( motion.key_poses[ no ], motion.key_poses[ no+1 ], s, p );
}


