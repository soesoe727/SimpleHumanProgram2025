/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作変形アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "Timeline.h"
#include "MotionDeformationEditApp.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  コンストラクタ
//
MotionDeformationEditApp::MotionDeformationEditApp() : MotionDeformationApp()
{
	app_name = "Motion Deformation";

	on_animation_mode = true;
	on_keypose_edit_mode = false;
}


//
//  デストラクタ
//
MotionDeformationEditApp::~MotionDeformationEditApp()
{
	if ( motion )
		delete  motion;
	if ( curr_posture && curr_posture->body )
		delete  curr_posture->body;
	if ( curr_posture )
		delete  curr_posture;
	if ( org_posture )
		delete  org_posture;
	if ( deformed_posture )
		delete  deformed_posture;
	if ( timeline )
		delete  timeline;
}


//
//  初期化
//
void  MotionDeformationEditApp::Initialize()
{
	MotionDeformationApp::Initialize();

	// IK計算の支点・末端関節の初期化
	base_joint_no = -1;
	ee_joint_no = -1;

	// 開始時のモードの設定
	on_animation_mode = true;
	on_keypose_edit_mode = false;
}


//
//  開始・リセット
//
void  MotionDeformationEditApp::Start()
{
	GLUTBaseApp::Start();
	
	// 変形動作再生モード
	if ( on_animation_mode )
	{
		// 変形動作再生モードの開始
		StartAnimationMode();
	}

	// キー姿勢編集モード
	if ( on_keypose_edit_mode )
	{
		// キー姿勢の初期化
		ResetKeypose();

		// キー姿勢編集モードの開始
		StartKeyposeEditMode();
	}
}


//
//  画面描画
//
void  MotionDeformationEditApp::Display()
{
	GLUTBaseApp::Display();

	// 変形動作再生モード
	if ( on_animation_mode )
	{
		// 動作変形後の姿勢を描画
		if ( deformed_posture )
		{
			glPushMatrix();

			glColor3f( 0.5f, 1.0f, 0.5f );
			DrawPosture( *deformed_posture );
			DrawPostureShadow( *deformed_posture, shadow_dir, shadow_color );

			glPopMatrix();
		}

		// 動作変形前の姿勢も描画（比較用）
		if ( draw_original_posture && org_posture )
		{
			glPushMatrix();

			if ( draw_postures_side_by_side )
				glTranslatef( -1.0f, 0.0f, 0.0f );

			glColor3f( 1.0f, 1.0f, 1.0f );
			DrawPosture( *org_posture );
			DrawPostureShadow( *org_posture, shadow_dir, shadow_color );

			glPopMatrix();
		}

		// 動作変形に使用するキー姿勢を描画（比較用）
		if ( draw_key_posture && draw_postures_side_by_side )
		{
			glPushMatrix();

			glTranslatef( 1.0f, 0.0f, 0.0f );

			glColor3f( 0.5f, 1.0f, 0.5f );
			DrawPosture( deformation.key_pose );
			DrawPostureShadow( deformation.key_pose, shadow_dir, shadow_color );

			glPopMatrix();
		}
	}

	// キー姿勢編集モード
	if ( on_keypose_edit_mode )
	{
		// IKによる姿勢編集中の姿勢を描画
		if ( curr_posture )
		{
			glColor3f( 1.0f, 1.0f, 1.0f );
			DrawPosture( *curr_posture );
			DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
		}

		// 視点が更新されたら関節点の位置を更新
		if ( curr_posture && is_view_updated )
		{
			// IK計算のための関節点の更新
			UpdateJointPositions( *curr_posture );

			// 視点の更新フラグをクリア
			is_view_updated = false;
		}

		// 関節点を描画
		if ( curr_posture && draw_joints )
		{
			DrawJoint();
		}
	}

	// タイムラインを描画
	if ( timeline )
	{
		timeline->SetLineTime( 1, animation_time );
		timeline->DrawTimeline();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Deformation" );
	char  message[64];
	if ( on_animation_mode )
		DrawTextInformation( 1, "Animation mode" );
	else
		DrawTextInformation( 1, "Kyepose edit mode" );
	if ( on_animation_mode && motion )
	{
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
		DrawTextInformation( 2, message );
	}
}


//
//  マウスクリック
//
void  MotionDeformationEditApp::MouseClick( int button, int state, int mx, int my )
{
	// キー姿勢編集モード中は、基底クラス（逆運動学計算クラス）の処理を呼び出し
	if ( on_keypose_edit_mode )
		InverseKinematicsCCDApp::MouseClick( button, state, mx, my );
	else
		GLUTBaseApp::MouseClick( button, state, mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 入力動作トラック上のクリック位置に応じて、変形動作の再生時刻を変更
	if ( drag_mouse_l && on_animation_mode && ( selected_track_no == 0 ) )
	{
		animation_time = selected_time;
		Animation( 0.0f );
	}

	// 動作変形トラック上のクリック位置に応じて、動作変形の時間範囲を変更
	if ( drag_mouse_l && ( selected_track_no == 1 ) )
	{
		// 動作変形の時間範囲を変更
		if ( selected_time < deformation.key_time )
			deformation.blend_in_duration = deformation.key_time - selected_time;
		else if ( selected_time > deformation.key_time )
			deformation.blend_out_duration = selected_time - deformation.key_time;

		// 動作変形の時間範囲をタイムラインに設定
		timeline->SetElementTime( 1, deformation.key_time - deformation.blend_in_duration, deformation.key_time + deformation.blend_out_duration );
	}
}


//
//  マウスドラッグ
//
void  MotionDeformationEditApp::MouseDrag( int mx, int my )
{
	// キー姿勢編集モード中は、基底クラス（逆運動学計算クラス）の処理を呼び出し
	if ( on_keypose_edit_mode )
	{
		InverseKinematicsCCDApp::MouseDrag( mx, my );

		// 逆運動学計算により変形された姿勢を、動作変形のキー姿勢として設定
		deformation.key_pose = *curr_posture;
	}
	else
		GLUTBaseApp::MouseDrag( mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 変形動作の再生時刻を変更
	if ( drag_mouse_l && on_animation_mode && ( selected_track_no == 0 ) )
	{
		animation_time = selected_time;
		drag_mouse_l = false;
		Animation( 0.0f );
		drag_mouse_l = true;
	}
}


//
//  キーボードのキー押下
//
void  MotionDeformationEditApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// 数字キーで入力動作・動作変形情報を変更
	if ( ( key >= '1' ) && ( key <= '9' ) )
	{
		InitMotion( key - '1' );
	}

	// スペースキーでモードを変更
	if ( key == ' ' )
	{
		// キー姿勢編集モードの開始
		if ( on_animation_mode )
		{
			on_animation_mode = false;
			on_keypose_edit_mode = true;

			StartKeyposeEditMode();
		}
		// 変形動作再生モードの開始
		else
		{
			on_animation_mode = true;
			on_keypose_edit_mode = false;
		}
	}

	// 変形動作再生モード中の操作
	if ( on_animation_mode )
	{
		// d キーで表示姿勢の変更
		if ( key == 'd' )
		{
			if ( !draw_original_posture )
			{
				draw_original_posture = true;
				draw_postures_side_by_side = false;
			}
			else if ( draw_original_posture && !draw_postures_side_by_side )
			{
				draw_key_posture = true;
				draw_postures_side_by_side = true;
			}
			else if ( draw_original_posture && draw_postures_side_by_side )
			{
				draw_key_posture = false;
				draw_original_posture = false;
			}
		}

		// s キーでアニメーションの停止・再開
		if ( key == 's' )
			on_animation = !on_animation;

		// w キーでアニメーションの再生速度を変更
		if ( key == 'w' )
			animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

		// n キーで次のフレーム
		if ( ( key == 'n' ) && !on_animation && motion )
		{
			on_animation = true;
			Animation( motion->interval );
			on_animation = false;
		}

		// p キーで前のフレーム
		if ( ( key == 'p' ) && !on_animation && motion && ( frame_no > 0 ) )
		{
			on_animation = true;
			Animation( - motion->interval );
			on_animation = false;
		}
	}

	// キー姿勢編集モード中の操作
	if ( on_keypose_edit_mode )
	{
		// v キーで関節点の描画の有無を変更
		if ( key == 'v' )
			draw_joints = !draw_joints;
	}

	// r キーで姿勢をリセット
	if ( key == 'r' )
		Start();

	// o キーで変形後の動作を保存
	if ( key == 'o' )
	{
		// 変形後の動作をBVH動作ファイルとして保存
		SaveDeformedMotionAsBVH( "deformed_motion.bvh" );
	}
}


//
//  アニメーション処理
//
void  MotionDeformationEditApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation_mode )
		return;

	// 基底クラスの処理を呼び出し
	 MotionDeformationApp::Animation( delta );
}


//
//  変形動作再生モードの開始
//
void  MotionDeformationEditApp::StartAnimationMode()
{
	// 時間・姿勢を初期化
	animation_time = 0.0f;
	frame_no = 0;
	Animation( 0.0f );
}


//
//  キー姿勢編集モードの開始
//
void  MotionDeformationEditApp::StartKeyposeEditMode()
{
	// 動作変形情報のキー姿勢を Inverse Kinematics を適用する現在姿勢として設定
	*curr_posture = deformation.key_pose;

	// IK計算のための関節点の更新
	UpdateJointPositions( *curr_posture );
}


//
// キー姿勢の初期化
//
void  MotionDeformationEditApp::ResetKeypose()
{
	motion->GetPosture( deformation.key_time, deformation.key_pose );
}


