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
#include "BVH.h"
#include "Timeline.h"
#include "MotionDeformationApp.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  コンストラクタ
//
MotionDeformationApp::MotionDeformationApp() : InverseKinematicsCCDApp()
{
	app_name = "Motion Deformation Base";

	motion = NULL;

	org_posture = NULL;
	deformed_posture = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	frame_no = 0;

	draw_original_posture = false;
	draw_postures_side_by_side = false;
	timeline = NULL;
}


//
//  デストラクタ
//
MotionDeformationApp::~MotionDeformationApp()
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
void  MotionDeformationApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// 入力動作・動作変形情報の初期化
	InitMotion( 0 );

	// 姿勢描画の設定
	draw_original_posture = true;
	draw_key_posture = true;
	draw_postures_side_by_side = false;

	// タイムライン描画機能の初期化
	timeline = new Timeline();
	if ( motion )
		InitTimeline( timeline, *motion, deformation, 0.0f );

	// 初期の視点を設定
	camera_yaw = -90.0f;
	camera_distance = 4.0f;
}


//
//  開始・リセット
//
void  MotionDeformationApp::Start()
{
	GLUTBaseApp::Start();
	
	on_animation = true;
	animation_time = 0.0f;
	frame_no = 0;
	Animation( 0.0f );
}


//
//  画面描画
//
void  MotionDeformationApp::Display()
{
	GLUTBaseApp::Display();

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

	// タイムラインを描画
	if ( timeline )
	{
		timeline->SetLineTime( 1, animation_time );
		timeline->DrawTimeline();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Deformation Base" );
	char  message[64];
	if ( motion )
	{
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
		DrawTextInformation( 1, message );
	}
}


//
//  ウィンドウサイズ変更
//
void  MotionDeformationApp::Reshape( int w, int h )
{
	GLUTBaseApp::Reshape( w, h );

	// タイムラインの描画領域の設定（画面の下部に配置）
	if ( timeline )
		timeline->SetViewAreaBottom( 0, 0, 0, 2, 32, 2 );
}


//
//  マウスクリック
//
void  MotionDeformationApp::MouseClick( int button, int state, int mx, int my )
{
	GLUTBaseApp::MouseClick( button, state, mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 入力動作トラック上のクリック位置に応じて、変形動作の再生時刻を変更
	if ( drag_mouse_l && ( selected_track_no == 0 ) )
	{
		animation_time = selected_time;
		Animation( 0.0f );
	}
}


//
//  マウスドラッグ
//
void  MotionDeformationApp::MouseDrag( int mx, int my )
{
	GLUTBaseApp::MouseDrag( mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 変形動作の再生時刻を変更
	if ( drag_mouse_l && ( selected_track_no == 0 ) )
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
void  MotionDeformationApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// 数字キーで入力動作・動作変形情報を変更
	if ( ( key >= '1' ) && ( key <= '9' ) )
	{
		InitMotion( key - '1' );
	}

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

	// r キーでリセット
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
void  MotionDeformationApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// マウスドラッグ中はアニメーションを停止
	if ( drag_mouse_l )
		return;

	// 動作データが読み込まれていなければ終了
	if ( !motion )
		return;

	// 時間を進める
	animation_time += delta * animation_speed;
	if ( animation_time > motion->GetDuration() )
		animation_time -= motion->GetDuration();

	// 現在のフレーム番号を計算
	frame_no = animation_time / motion->interval;

	// 動作データから現在時刻の姿勢を取得
	motion->GetPosture( animation_time, *org_posture );

	// 動作変形（動作ワーピング）の適用後の姿勢の計算
	ApplyMotionDeformation( animation_time, deformation, *org_posture, *deformed_posture );
}


//
//  入力動作・動作変形情報の初期化
//
void  MotionDeformationApp::InitMotion( int no )
{
	// テストケース1
	if ( no == 0 )
	{
		// サンプルBVH動作データを読み込み
		LoadBVH( "fight_punch.bvh" );
		if ( !motion )
			return;

		// キー姿勢のBHV動作データを読み込み
		BVH *  key_bvh = new BVH( "fight_punch_key.bvh" );

		// キー姿勢のBVH動作データにもとづいて、変形情報（キー姿勢）を設定
		if ( key_bvh->IsLoadSuccess() )
		{
			// BVH動作から動作データを生成
			Motion *  key_pose_motion = CoustructBVHMotion( key_bvh );

			// 動作変形（動作ワーピング）情報の初期化（キー時刻を指定）
			InitDeformationParameter( *motion, 0.80f, 0.70f, 0.50f, deformation );

			// 動作変形（動作ワーピング）情報のキー姿勢を設定
			deformation.key_pose = *key_pose_motion->GetFrame( 0 );
			deformation.key_pose.body = motion->body;

			// 使用済みデータの削除
			delete  key_pose_motion;
		}

		// 逆運動学計算を使用して、変形情報（キー姿勢）を設定
		else
		{
			// 動作変形（動作ワーピング）情報の初期化（キー時刻＋キー姿勢の右手の目標位置を指定）
			InitDeformationParameter( *motion, 0.80f, 0.70f, 0.50f, 0, 15, Vector3f( 0.0f, -0.2f, 0.0f ), deformation );
		}

		// 使用済みデータの削除
		delete  key_bvh;
	}

	// 以下、他のテストケースを追加する
	else if ( no == 1 )
	{
	}
}


//
//  動作変形情報にもとづくタイムラインの初期化
//
void  MotionDeformationApp::InitTimeline( Timeline * timeline, const Motion & motion, const MotionWarpingParam & deform, float curr_time )
{
	// タイムラインの時間範囲を設定
	timeline->SetTimeRange( -0.25f, motion.GetDuration() + 0.25f );

	// 全要素・縦棒の情報をクリア
	timeline->DeleteAllElements();
	timeline->DeleteAllLines();

	// 変形前の動作を表す要素を設定
	timeline->AddElement( 0.0f, motion.GetDuration(), Color4f( 1.0f, 1.0f, 1.0f, 1.0f ), motion.name.c_str(), 0 );

	// 動作変形の範囲を表す要素を設定
	timeline->AddElement( deform.key_time - deform.blend_in_duration, deform.key_time + deform.blend_out_duration, 
		Color4f( 0.5f, 1.0f, 0.5f, 1.0f ), "deformation", 1 );

	// 動作変形のキー時刻を表す縦線を設定
	timeline->AddLine( deform.key_time, Color4f( 1.0f, 0.0f, 0.0f, 1.0f ) );

	// 動作再生時刻を表す縦線を設定
	timeline->AddLine( curr_time, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ) );
}


//
//  BVH動作ファイルの読み込み、骨格・姿勢の初期化
//
void  MotionDeformationApp::LoadBVH( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );

	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion )
		return;

	// 骨格・動作・姿勢の削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;
	if ( org_posture )
		delete  org_posture;
	if ( deformed_posture )
		delete  deformed_posture;
 
	// 動作変形に使用する動作・姿勢の初期化
	motion = new_motion;
	curr_posture = new Posture();
	InitPosture( *curr_posture, motion->body );
	org_posture = new Posture();
	InitPosture( *org_posture, motion->body );
	deformed_posture = new Posture();
	InitPosture( *deformed_posture, motion->body );
}


//
//  変形後の動作をBVH動作ファイルとして保存
//
void  MotionDeformationApp::SaveDeformedMotionAsBVH( const char * file_name )
{
	// 動作変形適用後の動作を生成
	Motion *  deformed_motion = GenerateDeformedMotion( deformation, *motion );

	// 動作変形適用後の動作を保存
	// ※省略（各自作成）


	// 変形適用後の動作を削除
	delete  deformed_motion;
}



//
//  動作変形情報にもとづく動作変形処理
//


//
//  動作変形（動作ワーピング）の情報の初期化
//
void  InitDeformationParameter( 
	const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	MotionWarpingParam & param )
{
	param.key_time = key_time;
	param.blend_in_duration = blend_in_duration;
	param.blend_out_duration = blend_out_duration;
	motion.GetPosture( param.key_time, param.org_pose );
	param.key_pose = param.org_pose;
}


//
//  動作変形（動作ワーピング）の情報の初期化
//
void  InitDeformationParameter( 
	const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	int base_joint_no, int ee_joint_no, Point3f ee_joint_translation, 
	MotionWarpingParam & param )
{
	InitDeformationParameter( motion, key_time, blend_in_duration, blend_out_duration, param );

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	vector< Point3f >  joint_position_frame_array;
	ForwardKinematics( param.key_pose, seg_frame_array, joint_position_frame_array );

	// 指定関節の目標位置
	Point3f  ee_pos;
	ee_pos = joint_position_frame_array[ ee_joint_no ];
	ee_pos.add( ee_joint_translation );

	// キー姿勢の指定部位の位置を移動
	ApplyInverseKinematicsCCD( param.key_pose, base_joint_no, ee_joint_no, ee_pos );
}


//
//  動作変形（動作ワーピング）の適用後の動作を生成
//
Motion *  GenerateDeformedMotion( const MotionWarpingParam & deform, const Motion & motion )
{
	Motion *  deformed = NULL;

	// 動作変形前の動作を生成
	deformed = new Motion( motion );

	// 各フレームの姿勢を変形
	for ( int i = 0; i < motion.num_frames; i++ )
		ApplyMotionDeformation( motion.interval * i, deform, motion.frames[ i ], deformed->frames[ i ] );

	// 動作変形後の動作を返す
	return  deformed;
}


//
//  動作変形（動作ワーピング）の適用後の姿勢の計算
// （変形適用の重み 0.0～1.0 を返す）
//
float  ApplyMotionDeformation( float time, const MotionWarpingParam & deform, const Posture & input_pose, Posture & output_pose )
{
	// 動作変形の範囲外であれば、入力姿勢を出力姿勢とする
	if ( ( time < deform.key_time - deform.blend_in_duration ) || 
	     ( time > deform.key_time + deform.blend_out_duration ) )
	{
		output_pose = input_pose;
		return  0.0f;
	}


	// ※ レポート課題

	// 姿勢変形（動作ワーピング）の重みを計算
	float  ratio = 0.5f;
//	ratio = ???;


	// 姿勢変形（２つの姿勢の差分（dest - src）に重み ratio をかけたものを元の姿勢 org に加える ）
	PostureWarping( input_pose, deform.org_pose, deform.key_pose, ratio, output_pose );

	return  ratio;
}


//
//  動作ワーピングの姿勢変形（２つの姿勢の差分（dest - src）に重み ratio をかけたものを元の姿勢 org に加える ）
//
void  PostureWarping( const Posture & org, const Posture & src, const Posture & dest, float ratio, Posture & p )
{
	// ３つの姿勢の骨格モデルが異なる場合は終了
	if ( ( org.body != src.body ) || ( src.body != dest.body ) || ( dest.body != p.body ) )
		return;

	// 骨格モデルを取得
	const Skeleton *  body = org.body;


	// ※ レポート課題

	// 各関節の回転を計算
	for ( int i = 0; i < body->num_joints; i++ )
	{
//		p.joint_rotations[ i ] = ???;
	}

	// ルートの向きを計算
//	p.root_ori = ???;

	// ルートの位置を計算
//	p.root_pos = ???;
}


