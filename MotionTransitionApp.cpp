/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作接続・遷移アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "MotionTransition.h"
#include "MotionTransitionApp.h"
#include "BVH.h"
#include "Timeline.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  サンプル動作セットの読み込み
//
const Skeleton *  LoadSampleMotions( vector< MotionInfo * > &  motion_list, const Skeleton * body )
{
	// アプリケーションのテストに使用する動作データの定義（BVHファイル名・キー時刻・描画色）
	// 各歩行動作の、右足が地面から離れる、右足が着く、左足が離れる、左足が着く、右足が離れるの５つのキー時刻を設定
	// 先頭のキー区間を開始時のブレンド区間とし、末尾のキー区間を終了時のブレンド区間とする
	const int  num_motions = 3;
	const int  num_keytimes = 5; 
	const char *  sample_motions[ num_motions ] = {
		"sample_walking1.bvh", 
		"sample_walking2.bvh", 
		"sample_walking3.bvh" };
	const float  sample_keytimes[ num_motions ][ num_keytimes ] = {
		{ 2.35f, 3.00f, 3.08f, 3.68f, 3.74f },
		{ 1.30f, 2.07f, 2.12f, 2.88f, 2.94f },
		{ 1.20f, 2.00f, 2.08f, 2.80f, 2.86f }
	};
	const char *  sample_base_segments[ num_motions ] = {
		"LeftAnkle",
		"LeftAnkle",
		"LeftAnkle"
	};
	const float  sample_orientations[ num_motions ][ 2 ] = {
		{ 180.0f, 180.0f },
		{ 180.0f, 180.0f },
		{ 180.0f, 180.0f }
	};
	const Color3f  sample_colors[] = {
		Color3f( 0.5f, 1.0f, 0.5f ), 
		Color3f( 0.5f, 0.5f, 1.0f ), 
		Color3f( 1.0f, 0.5f, 0.5f ) 
	};

	// 動作のメタ情報
	MotionInfo *  info = NULL;

	// 動作データの読み込み、動作情報の設定
	for ( int i=0; i<num_motions; i++ )
	{
		// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
		Motion *  new_motion = LoadAndCoustructBVHMotion( sample_motions[ i ], body );

		// 動作データの読み込みに失敗したらスキップ
		if ( !new_motion )
			continue;

		// 最初に読み込んだ動作データの骨格モデルを記録（次以降の動作データの骨格モデルとして使用）
		if ( !body )
			body = new_motion->body;

		// 動作のメタ情報の設定
		info = new MotionInfo();
		InitMotionInfo( info, new_motion );
		for ( int j = 0; j < num_keytimes; j++ )
			info->keytimes.push_back( sample_keytimes[ i ][ j ] );
		info->begin_time = info->keytimes[ 0 ];
		info->blend_end_time = info->keytimes[ 1 ];
		info->blend_begin_time = info->keytimes[ info->keytimes.size() - 2 ];
		info->end_time = info->keytimes[ info->keytimes.size() - 1 ];
		info->base_segment_no = FindSegment( new_motion->body, sample_base_segments[ i ] );
		info->enable_ori = true;
		info->begin_ori = sample_orientations[ i ][ 0 ];
		info->end_ori = sample_orientations[ i ][ 1 ];
		info->color = sample_colors[ i ];

		// 動作リストに追加
		motion_list.push_back( info );
	}

	return  body;
}



//
//  コンストラクタ
//
MotionTransitionApp::MotionTransitionApp()
{
	app_name = "Motion Transition";

	transition = NULL;
	enable_transition = true;

	curr_posture = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;

	figure_color.set( 1.0f, 1.0f, 1.0f );
	timeline = NULL;
}


//
//  デストラクタ
//
MotionTransitionApp::~MotionTransitionApp()
{
	for ( int i=0; motion_list.size(); i++ )
	{
		delete  motion_list[ i ]->motion;
		delete  motion_list[ i ];
	}
	motion_list.clear();

	if ( transition )
		delete  transition;

	if ( curr_posture->body )
		delete  curr_posture->body;
	if ( curr_posture )
		delete  curr_posture;

	if ( timeline )
		delete  timeline;
}


//
//  初期化
//
void  MotionTransitionApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// サンプル動作セットの骨格モデル
	const Skeleton *  body = NULL;

	// サンプル動作セットの読み込み
	if ( motion_list.size() == 0 )
		body = LoadSampleMotions( motion_list );

	// 姿勢の初期化
	if ( body )
	{
		if ( curr_posture )
			delete  curr_posture;
		curr_posture = new Posture( body );
		InitPosture( *curr_posture, body );
	}

	// タイムライン描画機能の初期化
	timeline = new Timeline();
}


//
//  開始・リセット
//
void  MotionTransitionApp::Start()
{
	GLUTBaseApp::Start();
	
	// 現在の動作を初期化（動作リストの先頭の動作を現在の動作とする）
	curr_motion_no = 0;
	const MotionInfo *  curr_motion_info = NULL;
	if ( motion_list.size() > 0 )
		curr_motion_info = motion_list[ curr_motion_no ];
	else
		curr_motion_no = -1;

	// 次の動作・待ち動作を初期化
	next_motion_no = -1;
	waiting_motion_no = 0;

	// 動作接続・遷移機能の生成
	if ( transition )
		delete  transition;
	if ( enable_transition )
		transition = new MotionTransition();
	else
		transition = new MotionConnection();

	// アニメーション再生の初期化
	animation_time = 0.0f;
	if ( curr_motion_info )
		figure_color = curr_motion_info->color;
	transition_count = 0;

	// 現在の動作の開始位置・向きと開始時刻を設定
	Point3f   init_pos( 0.0f, 0.0f, 0.0f );
	Matrix3f  init_ori;
	init_ori.rotY( 180.0f * M_PI / 180.0f );
	curr_motion_mat.set( init_ori, init_pos, 1.0f );
	curr_start_time = animation_time;

	// アニメーション処理（開始時の姿勢の取得）
	Animation( 0.0f );
}


//
//  画面描画
//
void  MotionTransitionApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( figure_color.x, figure_color.y, figure_color.z );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// タイムラインを描画
	if ( timeline )
		timeline->DrawTimeline();

	// 現在のモード、現在・次の再生動作、アニメーション再生時間、動作接続・遷移の設定を表示
	char  message[ 64 ];
	DrawTextInformation( 0, "Motion Transition" );
	if ( curr_motion_no != -1 )
	{
		if ( ( next_motion_no != -1 ) && ( next_motion_no != curr_motion_no ) )
			sprintf( message, "%s -> %s", motion_list[ curr_motion_no ]->motion->name.c_str(), motion_list[ next_motion_no ]->motion->name.c_str() );
		else
			sprintf( message, "%s", motion_list[ curr_motion_no ]->motion->name.c_str() );
		DrawTextInformation( 1, message );
	}
	sprintf( message, "%.2f", animation_time );
	DrawTextInformation( 2, message );
	if ( !enable_transition )
		DrawTextInformation( 3, "Transition: Off" );
}


//
//  ウィンドウサイズ変更
//
void  MotionTransitionApp::Reshape( int w, int h )
{
	GLUTBaseApp::Reshape( w, h );

	// タイムラインの描画領域の設定（画面の下部に配置）
	if ( timeline )
		timeline->SetViewAreaBottom( 0, 0, 0, 3, 32, 2 );
}


//
//  マウスクリック
//
void  MotionTransitionApp::MouseClick( int button, int state, int mx, int my )
{
	GLUTBaseApp::MouseClick( button, state, mx, my );

	// 左ボタンが押されたら、次の再生動作を変更
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
	{
		int  motion_no = ( curr_motion_no + 1 ) % 3;
		SetNextMotion( motion_no );
	}
}


//
//  キーボードのキー押下
//
void  MotionTransitionApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// 数字キーで次に実行する動作を指定
	if ( ( key >= '1' ) && ( key <= '9' ) )
	{
		int  no = key - '1';
		SetNextMotion( no );
	}

	// s キーでアニメーションの停止・再開
	if ( key == 's' )
		on_animation = !on_animation;

	// w キーでアニメーションの再生速度を変更
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

	// n キーで次のフレーム
	if ( ( key == 'n' ) && !on_animation && transition && transition->GetPrevMotion() )
	{
		on_animation = true;
		Animation( transition->GetPrevMotion()->motion->interval );
		on_animation = false;
	}

	// d キーで動作接続・遷移の設定を変更
	if ( key == 'd' )
	{
		enable_transition = !enable_transition;
		Start();
	}
}


//
//  キーボードの特殊キー押下
//
void  MotionTransitionApp::KeyboardSpecial( unsigned char key, int mx, int my )
{
	// カーソル上キーが押されたら、次の再生動作を変更
	if ( key == GLUT_KEY_UP )
	{
		SetNextMotion();
	}
}


//
//  アニメーション処理
//
void  MotionTransitionApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	//  動作再生処理（動作接続・遷移を考慮）
	AnimationWithMotionTransition( delta );

	// 注視点を更新
	view_center.set( curr_posture->root_pos.x, 0.0f, curr_posture->root_pos.z );
}


//
//  次の動作を変更
//
void  MotionTransitionApp::SetNextMotion( int no )
{
	if ( motion_list.size() == 0 )
		return;

	// 次の動作が指定されなかった場合は、現在の再生動作の次の番号の動作を次の動作とする
	if ( no < 0 )
		no = ( curr_motion_no + 1 ) % motion_list.size();
	else
		no = no % motion_list.size();

	// 再生待ちの動作に設定
	waiting_motion_no = no;
}


//
//  動作再生処理（動作接続・遷移を考慮）
//
void  MotionTransitionApp::AnimationWithMotionTransition( float delta )
{
	// 現在の動作の情報を取得
	MotionInfo *  curr_motion_info = NULL;
	if ( curr_motion_no!= -1 )
		curr_motion_info = motion_list[ curr_motion_no ];

	// 次の動作の情報を取得
	MotionInfo *  next_motion_info = NULL;
	if ( next_motion_no != -1 )
		next_motion_info = motion_list[ next_motion_no ];

	// 次の動作が未設定であれば、実行待ち動作を次の動作とする
	else if ( waiting_motion_no != -1 )
	{
		// 実行待ちの動作の情報を取得
		next_motion_no = waiting_motion_no;
		next_motion_info = motion_list[ next_motion_no ];

		// 実行待ちの動作を初期化
		waiting_motion_no = -1;
	}

	// 動作接続・遷移の初期化
	//（前後の動作や前の動作の開始時刻が変更されたら、初期化を行う）
	if ( next_motion_info &&
		 ( transition->GetPrevMotion() != curr_motion_info ) ||
		 ( transition->GetNextMotion() != next_motion_info ) ||
		 ( transition->GetPrevBeginTime() != curr_start_time ) )
	{
		// 動作接続・遷移の初期化
		transition->Init( curr_motion_info, next_motion_info, curr_motion_mat, curr_start_time );

		// タイムラインへの前後の動作・遷移区間の設定
		InitTimeline( *timeline, *transition, transition_count );
	}


	// アニメーションの時間を進める
	animation_time += delta * animation_speed;

	// 動作接続・遷移の姿勢計算
	MotionTransition::MotionTransitionState  state = MotionTransition::MT_NONE;
	state = transition->GetPosture( animation_time, curr_posture );
	float  blend_ratio = transition->GetLastBlendRatio();

	// 姿勢の描画色を設定
	if ( ( blend_ratio > 0.0f ) && curr_motion_info && next_motion_info )
		figure_color.scaleAdd( blend_ratio, next_motion_info->color - curr_motion_info->color, curr_motion_info->color );
	else if ( curr_motion_info )
		figure_color = curr_motion_info->color;

	// タイムラインへの現在時刻・時間範囲の設定
	UpdateTimeline( *timeline, animation_time );


	// 動作接続・遷移の動作ブレンドの完了処理
	if ( state == MotionTransition::MT_NEXT_MOTION )
	{
		// 次の動作の変換行列と開始時刻を取得
		curr_motion_mat = transition->GetNextMotionMatrix();
		curr_start_time = transition->GetNextBeginTime();

		// 現在の動作を次の動作に切り替え
		curr_motion_no = next_motion_no;

		// 次の動作を初期化
		next_motion_no = -1;

		// 実行待ち動作が未設定であれば、現在の動作を繰り返すように設定する
		if ( waiting_motion_no == -1 )
			waiting_motion_no = curr_motion_no;

		// 動作遷移回数のカウントを加算（タイムライン表示用）
		transition_count ++;
	}
}


//
//  タイムラインへの現在動作・次の動作・遷移区間の設定
//
void  MotionTransitionApp::InitTimeline( Timeline & timeline, const MotionTransition & trans, int transition_count )
{
	Color4f  prev_color, next_color, trans_color;

	const MotionInfo *  prev_motion = trans.GetPrevMotion();
	const MotionInfo *  next_motion = trans.GetNextMotion();

	// 動作の開始時刻（グローバル時刻）
	float  prev_begin_time = trans.GetPrevBeginTime();
	float  next_begin_time = trans.GetNextBeginTime();

	// タイムラインに各動作を表す要素を追加
	while( timeline.GetNumElements() < 3 )
		timeline.AddElement( 0.0f, 1.0f, Color4f( 1.0f, 1.0f, 1.0f, 1.0f ), NULL, timeline.GetNumElements() );

	// 前の動作・次の動作を配置するトラック番号を決定
	int  prev_track_no = 0;
	int  next_track_no = 2;
	if ( transition_count % 2 == 1 )
	{
		prev_track_no = 2;
		next_track_no = 0;
	}

	// 前の動作の要素の情報を設定
	if ( prev_motion )
	{
		timeline.SetElementEnable( 0, true );
		timeline.SetElementTime( 0, prev_begin_time, 
			prev_begin_time + ( prev_motion->end_time - prev_motion->begin_time ) );
		prev_color.set( prev_motion->color.x, prev_motion->color.y, prev_motion->color.z, 1.0f );
		timeline.SetElementColor( 0, prev_color );
		timeline.SetElementText( 0, prev_motion->motion->name.c_str() );
		timeline.SetElementTrackNo( 0, prev_track_no );
	}
	else
	{
		timeline.SetElementEnable( 0, false );
	}

	// 遷移後の動作の要素の情報を設定
	if ( next_motion )
	{
		timeline.SetElementEnable( 2, true );
		timeline.SetElementTime( 2, next_begin_time, 
			next_begin_time + ( next_motion->end_time - next_motion->begin_time ) );
		next_color.set( next_motion->color.x, next_motion->color.y, next_motion->color.z, 1.0f );
		timeline.SetElementColor( 2, next_color );
		timeline.SetElementText( 2, next_motion->motion->name.c_str() );
		timeline.SetElementTrackNo( 2, next_track_no );
	}
	else
	{
		timeline.SetElementEnable( 2, false );
	}

	// 遷移区間の要素の情報を設定
	if ( next_motion )
	{
		timeline.SetElementEnable( 1, true );
		timeline.SetElementTime( 1, trans.GetBlendBeginTime(), trans.GetBlendEndTime() );
		timeline.SetElementColor( 1, prev_color, next_color );
		timeline.SetElementText( 1, "transition" );
		timeline.SetElementTrackNo( 1, 1 );
	}
	else
	{
		timeline.SetElementEnable( 1, false );
	}

	// 各動作のブレンド区間の表示に必要な区間を初期化
	int  size = 4;
	while( timeline.GetNumSubElements() < size )
		timeline.AddSubElement( 0, 0.0f, 1.0f, 0.0f, 1.0f, Color4f( 1.0f, 1.0f, 1.0f, 1.0f ) );

	// 各動作の開始・終了ブレンド区間の要素の情報を設定
	float  base_time;
	int  count = 0;
	if ( prev_motion )
	{
		base_time = prev_begin_time - prev_motion->begin_time;
		prev_color.set( prev_motion->color.x + 0.1f, prev_motion->color.y + 0.1f, prev_motion->color.z + 0.1f, 1.0f );
		timeline.SetSubElementEnable( count, true );
		timeline.SetSubElementParent( count, 0 );
		timeline.SetSubElementTime( count, base_time + prev_motion->begin_time, base_time + prev_motion->blend_end_time );
		timeline.SetSubElementColor( count, prev_color );
		count ++;
		timeline.SetSubElementEnable( count, true );
		timeline.SetSubElementParent( count, 0 );
		timeline.SetSubElementTime( count, base_time + prev_motion->blend_begin_time, base_time + prev_motion->end_time );
		timeline.SetSubElementColor( count, prev_color );
		count ++;
	}

	if ( next_motion )
	{
		base_time = next_begin_time - next_motion->begin_time;
		next_color.set( next_motion->color.x + 0.1f, next_motion->color.y + 0.1f, next_motion->color.z + 0.1f, 1.0f );
		timeline.SetSubElementEnable( count, true );
		timeline.SetSubElementParent( count, 2 );
		timeline.SetSubElementTime( count, base_time + next_motion->begin_time, base_time + next_motion->blend_end_time );
		timeline.SetSubElementColor( count, next_color );
		count ++;
		timeline.SetSubElementEnable( count, true );
		timeline.SetSubElementParent( count, 2 );
		timeline.SetSubElementTime( count, base_time + next_motion->blend_begin_time, base_time + next_motion->end_time );
		timeline.SetSubElementColor( count, next_color );
		count ++;
	}
}


//
//  タイムラインへの現在時刻の設定
//
void  MotionTransitionApp::UpdateTimeline( Timeline & timeline, float curr_time )
{
	// 現在時刻のラインの情報を設定
	while( timeline.GetNumLines() < 1 )
		timeline.AddLine( curr_time, Color4f( 1.0f, 0.0f, 0.0f, 1.0f ) );
	timeline.SetLineTime( 0, curr_time );

	// 描画時間範囲の情報を設定
	timeline.SetTimeRange( curr_time - 2.0f, curr_time + 3.0f );
}


