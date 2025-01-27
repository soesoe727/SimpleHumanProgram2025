/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作再生アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "MotionPlaybackApp.h"
#include "Timeline.h"
#define  _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>


//
//  コンストラクタ
//
MotionPlaybackApp::MotionPlaybackApp()
{
	app_name = "Motion Playback";

	motion = NULL;
	motion2 = NULL;
	curr_posture = NULL;
	curr_posture2 = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	frame_no = 0;
	area= -1;
	trace = -1;
	view_segment = 12;
	DTWa = NULL;
	flag = 1;
	pattern = 0;
	timeline = NULL;
}


//
//  デストラクタ
//
MotionPlaybackApp::~MotionPlaybackApp()
{
	// 骨格・動作・姿勢の削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// 骨格・動作・姿勢の削除
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;

	if(DTWa)
		delete DTWa;
	if(timeline)
		delete timeline;
}


//
//  初期化
//
void  MotionPlaybackApp::Initialize()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Initialize();
	DTWa = new DTWinformation();
	// サンプルBVH動作データを読み込み
	//LoadBVH( "sample_walking1.bvh" );
	OpenNewBVH();
	OpenNewBVH2();
	timeline = new Timeline();
	if(motion && motion2)
	{
		//腰の位置を最初に一緒にする
		Point3f root = motion2->frames[0].root_pos - motion->frames[0].root_pos;
		for(int i = 0; i < motion2->num_frames; i++)
			motion2->frames[i].root_pos -= root;

		InitSegmentname(motion->body->num_segments);
		PatternTimeline(timeline, * motion, frame_no, DTWa);
	}	
}

//
//  開始・リセット
//
void  MotionPlaybackApp::Start()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Start();

	// アニメーション再生のための変数の初期化
	on_animation = true;
	animation_time = 0.0f;
	frame_no = 0;

	// アニメーション再生処理（姿勢の初期化）
	Animation( 0.0f );

	//カメラ位置の調整
	view_center = motion->frames[0].root_pos;
}


//
//  画面描画
//
void  MotionPlaybackApp::Display()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Display();

	// キャラクタを描画
	
	if(curr_posture && curr_posture2)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		

		if(sabun_flag == -1)
		{
			//1人目の動作の可視化(色付け)
			Pass_posture1 = motion->frames[DTWa->PassAll[0][frame_no]];
			DrawPostureColor(Pass_posture1, pattern, view_segment);
			DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);

			//2人目の動作の可視化(白･灰)(パス対応)
			Pass_posture2 = motion2->frames[DTWa->PassAll[1][frame_no]];
			DrawPostureGray(Pass_posture2, pattern, view_segment);
			DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
		}
		else
		{
			//1人目の動作の可視化(色付け)
			Pass_posture1 = motion->frames[frame_no + m1f];
			DrawPostureColor(Pass_posture1, pattern, view_segment);
			DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);

			//2人目の動作の可視化(白･灰)(パス対応)
			Pass_posture2 = motion2->frames[frame_no + m2f];
			DrawPostureGray(Pass_posture2, pattern, view_segment);
			DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
		}

		if(area == 1)
		{
			//一人目の動作の部位の領域変化(オレンジ)
			glColor4f( 1.0f, 0.6f, 0.0f, 0.1f );
			DrawPart(motion, view_segment);
			//一人目の動作の部位の領域変化(水)
			glColor4f( 0.7f, 0.8f, 0.9f, 0.1f );
			DrawPart(motion2, view_segment);	
		}

		if(trace == 1)//軌跡の表示変化
		{
			glColor3f(1.0f, 0.0f, 0.0f);
			DrawTrace(motion, view_segment);
			glColor3f(0.0f, 0.0f, 1.0f);
			DrawTrace(motion2, view_segment);
		}	

		glDisable(GL_BLEND);
	}

	// タイムラインを描画
	if ( timeline )
	{
		int Track_num = 0;
		int name_space = 30.0f;
		
		
		//部位の集合ごとの色付け
		if(pattern == 0 )
		{
			for (int j = 12; j >= 11; j--)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
			for (int j = 10; j >= 7; j--)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
			timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, 0));
			for (int j = 13; j <= 16; j++)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
			for (int j = 36; j <= 39; j++)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
			for (int j = 1; j <= 3; j++)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
			for (int j = 4; j <= 6; j++)
				timeline->SetElementColor(Track_num * DTWa->DTWframe + Track_num++, Pattern_Color(pattern, j));
		}
		else
		{
			for (int j = 12; j >= 11; j--)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
			for (int j = 10; j >= 7; j--)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
			timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, 0));
			for (int j = 13; j <= 16; j++)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
			for (int j = 36; j <= 39; j++)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
			for (int j = 1; j <= 3; j++)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
			for (int j = 4; j <= 6; j++)
				timeline->SetElementColor(Track_num * motion->num_frames + Track_num++, Pattern_Color(pattern, j));
		}
		PatternTimeline(timeline, * motion, frame_no, DTWa);

		timeline->SetLineTime( 1, animation_time / motion->interval + name_space );
		timeline->DrawTimeline();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Playback" );
	char  message1[64];
	char  message2[64];
	char message3[64];
	char messages[5][64]{};
	if ( motion && motion2 )
	{
		sprintf( message1, "%.2f (%d)", animation_time, frame_no );
		sprintf( message2, "viewing %s", motion->body->segments[view_segment]->name.c_str() );
		for(int i = 0; i < 5; i++)
			sprintf( messages[i], "%s : %.2f percent", motion->body->segments[DTWa->DTWorder[i]]->name.c_str(), 100 * DTWa->DisTotalPart[DTWa->DTWorder[i]] / DTWa->DisTotalAll);
		/*
		if( pattern == 0 )
			sprintf(message3, "leg:%.2f, chest:%.2f, arm:%.2f", DTWa->left_leg + DTWa->right_leg, DTWa->chest + DTWa->head, DTWa->left_arm + DTWa->right_arm);
		else if( pattern == 1 )
			sprintf(message3, "right_leg:%.2f, left_leg:%.2f", DTWa->right_leg, DTWa->left_leg);
		else if(pattern == 2)
			sprintf(message3, "chest:%.2f, head:%.2f", DTWa->chest, DTWa->head);
		else
			sprintf(message3, "right_arm:%.2f, left_arm:%.2f", DTWa->right_arm, DTWa->left_arm);
		*/
		DrawTextInformation( 1, message1 );
		DrawTextInformation( 2, message2 );
		for(int i = 0; i < 5; i++)
			DrawTextInformation( 3+i, messages[i] );
		//DrawTextInformation( 8, message3 );
	}
	else
	{
		sprintf( message1, "Press 'L' key to Load BVH files" );
		DrawTextInformation( 1, message1 );
	}
	
}

//
//  ウィンドウサイズ変更
//
void MotionPlaybackApp::Reshape(int w, int h)
{
	GLUTBaseApp::Reshape( w, h );

	// タイムラインの描画領域の設定（画面の下部に配置）
	if ( timeline )
		timeline->SetViewAreaBottom( 0, 1, 0, 26, 10, 2 );
}

//
//  キーボードのキー押下
//
void  MotionPlaybackApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );
	int a;
	//f
	if(key == 'f')
	{
		if(sabun_flag == -1)
		{
			m1f = DTWa->PassAll[0][frame_no];
			m2f = DTWa->PassAll[1][frame_no];
		}
		mf = frame_no;
		sabun_flag *= -1;
		frame_no = 0;
		std::cout << m1f << " " << m2f << " " << mf << std::endl;
	}
	// s キーでアニメーションの停止・再開
	if ( key == ' ' )
		on_animation = !on_animation;

	// w キーでアニメーションの再生速度を変更
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

	// . キーで次のフレーム
	if ( ( key == '.' ) && !on_animation && motion )
	{
		on_animation = true;
		Animation( motion->interval );
		on_animation = false;
	}

	// , キーで前のフレーム
	if ( ( key == ',' ) && !on_animation && motion && ( frame_no > 0 ) )
	{
		on_animation = true;
		Animation( - motion->interval );
		on_animation = false;
	}
	
	// L キーで再生動作の変更
	//if ( key == 'L' )
	//{
	//	// ファイルダイアログを表示してBVHファイルを選択・読み込み
	//	OpenNewBVH();
	//	OpenNewBVH2();
	//}

	// a キーで領域の表示変更
	if ( key == 'a' )
		area *= -1;

	// q キーで表示する領域の変更-
	if ( key == 'q' )
		view_segment = ViewSegment(-1, view_segment);

	// e キーで表示する領域の変更+
	if ( key == 'e' )
		view_segment = ViewSegment(1, view_segment);

	// t キーで軌跡の表示変更
	if ( key == 't' )
		trace *= -1;

	// f キーでフレーム毎誤差表示
	//if ( key == 'f' )
	//	flag *= -1;

	// 0 キーでパターン0
	if ( key == '0' )
		pattern = 0;
	// 1 キーでパターン1
	if ( key == '1' )
		pattern = 1;
	// 2 キーでパターン2
	if ( key == '2' )
		pattern = 2;
	// 3 キーでパターン3
	if ( key == '3' )
		pattern = 3;
	// 4 キーでパターン4
	if ( key == '4' )
		pattern = 4;
	// 5 キーでパターン5
	if ( key == '5' )
		pattern = 5;
}

//
//  マウスドラッグ
//
void  MotionPlaybackApp::MouseDrag( int mx, int my )
{
	GLUTBaseApp::MouseDrag( mx, my );
	int name_space = 30.0f;

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if(selected_time < 0)
		selected_time = 0.0f;
	else if(selected_time > DTWa->DTWframe - 1)
		selected_time = DTWa->DTWframe - 1;

	// 変形動作の再生時刻を変更
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		drag_mouse_l = false;
		Animation( 0.0f );
		drag_mouse_l = true;
	}
}

//
//  マウスクリック
//
void MotionPlaybackApp::MouseClick(int button, int state, int mx, int my)
{
	GLUTBaseApp::MouseClick( button, state, mx, my );
	int name_space = 30.0f;

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if(selected_time < 0)
		selected_time = 0.0f;
	else if(selected_time > DTWa->DTWframe - 1)
		selected_time = DTWa->DTWframe - 1;

	// 入力動作トラック上のクリック位置に応じて、変形動作の再生時刻を変更
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		Animation( 0.0f );
	}
}

//
//  アニメーション処理
//
void  MotionPlaybackApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// マウスドラッグ中はアニメーションを停止
	if ( drag_mouse_l )
		return;

	// 動作データが読み込まれていなければ終了
	if ( !motion || !motion2 )
		return;

	// 時間を進める
	animation_time += delta * animation_speed;
	if ( pattern == 5 && animation_time > motion->GetDuration() )
		animation_time -= motion->GetDuration();
	else if( sabun_flag == -1 && animation_time > DTWa->DTWframe * motion->interval)
		animation_time -= DTWa->DTWframe * motion->interval;
	else if( sabun_flag == 1 && animation_time > (motion2->num_frames - m2f) * motion->interval)
		animation_time -= (motion2->num_frames - m2f) * motion->interval;
	// 現在のフレーム番号を計算
	frame_no = animation_time / motion->interval;

	// 動作データから現在時刻の姿勢を取得
	motion->GetPosture( animation_time, *curr_posture );
	motion2->GetPosture( animation_time, *curr_posture2 );

}



//
//  補助処理
//

//
//view_segmentの順番
//
int  MotionPlaybackApp::ViewSegment(int i, int seg)
{
	if(i > 0)
	{
		if(seg == 7)
			return 0;
		else if(seg == 0)
			return 13;
		else if(seg == 16)
			return 36;
		else if(seg == 39)
			return 1;
		else if(seg == 6)
			return 12;
		else if(seg <= 12 && seg >= 8)
			return seg - 1;
		else
			return seg + 1;
	}
	else
	{
		if(seg == 12)
			return 6;
		else if(seg == 0)
			return 7;
		else if(seg == 13)
			return 0;
		else if(seg == 36)
			return 16;
		else if(seg == 1)
			return 39;
		else if(seg <= 11 && seg >= 7)
			return seg + 1;
		else
			return seg - 1;
	}
}

//
//  BVH動作ファイルの読み込み、動作・姿勢の初期化
//
void  MotionPlaybackApp::LoadBVH( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );
	
	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion )
		return;
	
	// 現在使用している骨格・動作・姿勢を削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// 動作再生に使用する動作・姿勢を初期化
	motion = new_motion;
	curr_posture = new Posture( motion->body );

	// 動作再生開始
	//Start();
}
void  MotionPlaybackApp::LoadBVH2( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion2 = LoadAndCoustructBVHMotion( file_name );
	
	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion2 )
		return;
	
	// 現在使用している骨格・動作・姿勢を削除
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;
	

	// 動作再生に使用する動作・姿勢を初期化
	motion2 = new_motion2;
	curr_posture2 = new Posture( motion2->body );
	
	
	//DTWの作成
	DTWa->DTWinformation_init( motion->num_frames, motion2->num_frames, *motion, *motion2 );
	
	// 動作再生の開始
	//Start();
}

//
//  ファイルダイアログを表示してBVH動作ファイルを選択・読み込み
//
void  MotionPlaybackApp::OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// ファイルダイアログの設定
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof( OPENFILENAME ) );
	open_file.lStructSize = sizeof( OPENFILENAME );
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select first BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if ( ret )
	{
		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH( file_name );

		// 動作再生の開始
		//Start();
	}
#endif // WIN32
}
void  MotionPlaybackApp::OpenNewBVH2()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// ファイルダイアログの設定
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof( OPENFILENAME ) );
	open_file.lStructSize = sizeof( OPENFILENAME );
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select second BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if ( ret )
	{
		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH2( file_name );

		// 動作再生の開始
		//Start();
	}
#endif // WIN32
}

//
//タイムラインのパターン毎読み込み
//
void MotionPlaybackApp::PatternTimeline(Timeline* timeline, Motion& motion, float curr_frame, DTWinformation* DTW)
{
	int Track_num = 0;
	
	// タイムラインの時間範囲を設定
	timeline->SetTimeRange( 0.0f, DTW->DTWframe + num_space );
	// 全要素・縦棒の情報をクリア
	timeline->DeleteAllElements();
	timeline->DeleteAllLines();
	timeline->DeleteAllSubElements();

	// j番目の誤差
	switch(pattern)
	{
		case 0:
			for(int j = 12; j >= 11; j--)//頭部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			for(int j = 10; j >= 7; j--)//胸部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			ColorBarElementSetPart(timeline, 0, Track_num++, DTW->head_chest, DTW->PassAll, motion);//腰
			Track_num++;

			for(int j = 13; j <= 16; j++)//右腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			for(int j = 36; j <= 39; j++)//左腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 1; j <= 3; j++)//右脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			for(int j = 4; j <= 6; j++)//左脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			break;

		case 1:
			for(int j = 12; j >= 11; j--)//頭部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head, DTW->PassAll, motion);
			Track_num++;

			for(int j = 10; j >= 7; j--)//胸部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->chest, DTW->PassAll, motion);
			ColorBarElementSetPart(timeline, 0, Track_num++, DTW->chest, DTW->PassAll, motion);//腰
			Track_num++;

			for(int j = 13; j <= 16; j++)//右腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			for(int j = 36; j <= 39; j++)//左腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 1; j <= 3; j++)//右脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			for(int j = 4; j <= 6; j++)//左脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			break;

		case 2:
			for(int j = 12; j >= 11; j--)//頭部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			for(int j = 10; j >= 7; j--)//胸部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			ColorBarElementSetPart(timeline, 0, Track_num++, DTW->head_chest, DTW->PassAll, motion);//腰
			Track_num++;

			for(int j = 13; j <= 16; j++)//右腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->right_arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 36; j <= 39; j++)//左腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->left_arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 1; j <= 3; j++)//右脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			for(int j = 4; j <= 6; j++)//左脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->leg, DTW->PassAll, motion);
			break;

		case 3:
			for(int j = 12; j >= 11; j--)//頭部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			for(int j = 10; j >= 7; j--)//胸部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head_chest, DTW->PassAll, motion);
			ColorBarElementSetPart(timeline, 0, Track_num, DTW->head_chest, DTW->PassAll, motion);//腰
			Track_num++;

			for(int j = 13; j <= 16; j++)//右腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			for(int j = 36; j <= 39; j++)//左腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 1; j <= 3; j++)//右脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->right_leg, DTW->PassAll, motion);
			Track_num++;

			for(int j = 4; j <= 6; j++)//左脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->left_leg, DTW->PassAll, motion);
			break;

		case 4:
			for(int j = 12; j >= 11; j--)//頭部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->head, DTW->PassAll, motion);
			Track_num++;

			for(int j = 10; j >= 7; j--)//胸部
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->chest, DTW->PassAll, motion);
			ColorBarElementSetPart(timeline, 0, Track_num++, DTW->chest, DTW->PassAll, motion);//腰
			Track_num++;

			for(int j = 13; j <= 16; j++)//右腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->right_arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 36; j <= 39; j++)//左腕
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->left_arm, DTW->PassAll, motion);
			Track_num++;

			for(int j = 1; j <= 3; j++)//右脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->right_leg, DTW->PassAll, motion);
			Track_num++;

			for(int j = 4; j <= 6; j++)//左脚
				ColorBarElementSetPart(timeline, j, Track_num++, DTW->left_leg, DTW->PassAll, motion);
			break;

		case 5:
			for(int j = 12; j >= 11; j--)//頭部
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			Track_num++;
			for(int j = 10; j >= 7; j--)//胸部
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			if(view_segment != 0)//腰
					ColorBarElementGray(timeline, 0, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, 0, Track_num, DTW->DistancePart[0], DTW->PassAll, motion);
			Track_num++;
			Track_num++;
			for(int j = 13; j <= 16; j++)//右腕
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			Track_num++;
			for(int j = 36; j <= 39; j++)//左腕
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			Track_num++;
			for(int j = 1; j <= 3; j++)//右脚
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			Track_num++;
			for(int j = 4; j <= 6; j++)//左脚
			{
				if(view_segment != j)
					ColorBarElementGray(timeline, j, Track_num, DTW->PassAll, motion);
				else
					ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->PassAll, motion);
				Track_num++;
			}
			break;
		default:
			break;
	}

	// 動作再生時刻を表す縦線を設定
	timeline->AddLine( curr_frame + num_space, Color4f( 1.0f, 1.0f, 1.0f, 1.0f ) );

	//timeline->AddLine( mf + num_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ) );
}

//
//パターンによるカラーバーの部位名前の色変化
//
Color4f MotionPlaybackApp::Pattern_Color(int pattern, int num_segment)
{
	Color4f c;

	switch (pattern)
	{
		case 0: //全体3色
			if( 1 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.2f, 1.0f );
			else if( num_segment <= 12 )
				c = Color4f( 0.2f, 0.8f, 0.2f, 1.0f );
			else
				c = Color4f( 0.2f, 0.2f, 0.8f, 1.0f );
			break;
		case 1: //頭部と胸部2色
			if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( 7 <= num_segment && num_segment <= 10 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( num_segment == 11 || num_segment == 12 )
				c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 2: //腕2色
			if( 13 <= num_segment && num_segment <= 35 )
				c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
			else if( 36 <= num_segment )
				c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 3: //脚2色
			if( 1 <= num_segment && num_segment <= 3 )
				c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
			else if( 4 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 4: //全体6色
			if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( 1 <= num_segment && num_segment <= 3 )
				c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
			else if( 4 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
			else if( 7 <= num_segment && num_segment <= 10 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( num_segment == 11 || num_segment == 12 )
				c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
			else if( 13 <= num_segment && num_segment <= 35 )
				c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
			else
				c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			break;
		case 5: //1部位1色
			if( num_segment == view_segment )
				if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
				else if( 1 <= num_segment && num_segment <= 3 )
					c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
				else if( 4 <= num_segment && num_segment <= 6 )
					c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
				else if( 7 <= num_segment && num_segment <= 10 )
					c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
				else if( num_segment == 11 || num_segment == 12 )
					c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
				else if( 13 <= num_segment && num_segment <= 35 )
					c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
				else
					c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		default:
			std::cout << "pattern isn't defined." << std::endl;
	}

	return c;
}

//
//体節の名前を入力
//
void MotionPlaybackApp::InitSegmentname(int num_segments)
{
	//体節の名前を入れる
	int i = 0;
	motion->body->segments[i++]->name = "Pelvis";
	motion->body->segments[i++]->name = "R-Thigh";
	motion->body->segments[i++]->name = "R-Calf";
	motion->body->segments[i++]->name = "R-Foof";
	motion->body->segments[i++]->name = "L-Thigh";
	motion->body->segments[i++]->name = "L-Calf";
	motion->body->segments[i++]->name = "L-Foof";
	motion->body->segments[i++]->name = "Back1";
	motion->body->segments[i++]->name = "Back2";
	motion->body->segments[i++]->name = "Back3";
	motion->body->segments[i++]->name = "Chest";
	motion->body->segments[i++]->name = "Neck";
	motion->body->segments[i++]->name = "Head";
	motion->body->segments[i++]->name = "R-Clavicle";
	motion->body->segments[i++]->name = "R-UpperArm";
	motion->body->segments[i++]->name = "R-ForeArm";
	motion->body->segments[i++]->name = "R-Hand";
	motion->body->segments[36]->name = "L-Clavicle";
	motion->body->segments[37]->name = "L-UpperArm";
	motion->body->segments[38]->name = "L-ForeArm";
	motion->body->segments[39]->name = "L-Hand";
}


//
//部位の集合毎のカラーバーの誤差による色の変化を設定(DTWframe)
//
void MotionPlaybackApp::ColorBarElementSetPart(Timeline* timeline, int segment_num, int Track_num, vector<float> Distance, vector<vector<int>> PassAll, Motion & motion)
{
	float red_ratio = 0.0f;
	float max_dep = 0.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	//frame内での最大誤差値を求める
	for(int f = 0; f < PassAll[0].size(); f++)
	{
		if(max_dep < Distance[PassAll[0][f]])
		{
			max_dep = Distance[PassAll[0][f]];
			max_frame = f;
		}
	}
	
	//std::cout << max_frame << std::endl;

	//誤差の大きさと色付けしてフレーム毎に表示
	for(int i = 0; i < PassAll[0].size(); i++)
	{
		if(i == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
		red_ratio = Distance[PassAll[0][i]] / max_dep;
		if(red_ratio > 0.6f)
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 1.0f, (1.0f - red_ratio) * (5.0f / 4.0f) * 2.0f, 0.0f, 1.0f ), "", Track_num );
		else
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( red_ratio * (5.0f / 6.0f) * 2.0f, 1.0f, 0.0f, 1.0f ), "", Track_num );

		if(i == max_frame && max_frame != -1)
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ), "", Track_num );
	}
}

//
//部位毎のカラーバーの誤差による色の変化を設定(DTWframe)
//
void MotionPlaybackApp::ColorBarElementPart(Timeline* timeline, int segment_num, int Track_num, vector<vector<float>> Distance, vector<vector<int>> PassAll, Motion& motion)
{
	float red_ratio = 0.0f;
	float max_dep = 0.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	//frame内での最大誤差値を求める
	for(int f = 0; f < PassAll[0].size(); f++)
	{
		if(max_dep < Distance[PassAll[0][f]][PassAll[1][f]])
		{
			max_dep = Distance[PassAll[0][f]][PassAll[1][f]];
			max_frame = f;
		}
	}
	
	//std::cout << max_frame << std::endl;

	//誤差の大きさと色付けしてフレーム毎に表示
	for(int i = 0; i < PassAll[0].size(); i++)
	{
		if(i == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
		red_ratio = Distance[PassAll[0][i]][PassAll[1][i]] / max_dep;
		if(red_ratio > 0.6f)
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 1.0f, (1.0f - red_ratio) * (5.0f / 4.0f) * 2.0f, 0.0f, 1.0f ), "", Track_num );
		else
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( red_ratio * (5.0f / 6.0f) * 2.0f, 1.0f, 0.0f, 1.0f ), "", Track_num );

		if(i == max_frame && max_frame != -1)
			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ), "", Track_num );
	}
}

//
//カラーバーを全て灰色に設定
//
void MotionPlaybackApp::ColorBarElementGray(Timeline* timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion & motion)
{
	float name_space = 30.0f;
				
	//名前だけ表示
	for(int i = 0; i < PassAll[0].size(); i++)
		if(i == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
}

//
//全部位との距離を取る
//
float MotionPlaybackApp::DistanceCalc(int num_segments, vector<Vector3f> v1, vector<Vector3f> v2)
{
	float pos_dis = 0.0f;
	for(int i = 0; i < num_segments; i++)
	{
		//手の部位は計算しない
		while(i > 16 && i < 36)
			i++;
		if(i > 39)
			break;

		pos_dis += sqrt(pow(v1[i].x - v2[i].x, 2.0) + pow(v1[i].y - v2[i].y, 2.0) + pow(v1[i].z - v2[i].z, 2.0));
	}

	return pos_dis;
}

//
//1部位との距離を取る
//
float MotionPlaybackApp::DistanceCalcPart(int frame1, int frame2, int num_segment, vector<vector<Vector3f>> v1, vector<vector<Vector3f>> v2)
{
	float pos_dis;
	pos_dis = sqrt(pow(v1[frame1][num_segment].x - v2[frame2][num_segment].x, 2.0) + pow(v1[frame1][num_segment].y - v2[frame2][num_segment].y, 2.0) + pow(v1[frame1][num_segment].z - v2[frame2][num_segment].z, 2.0));
	return pos_dis;
}


//
//DTW初期化
//
void DTWinformation::DTWinformation_init( int frames1, int frames2, const Motion & motion1, const Motion & motion2 )
{
	//iが体節、jがフレーム1、kがフレーム2
	int Num_segments = motion1.body->num_segments;
	this->DistanceAll.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->DistancePart.resize(Num_segments,vector<vector<float>>(frames1 + 1, vector<float>(frames2 + 1, 0.0f)));
	this->DTWorder = new int[Num_segments];
	this->DTWforder = new int[Num_segments];
	this->DisTotalAll = 0.0f;
	this->DisTotalPart = new float [Num_segments];

	float min_distance = 100.0f;
	vector< Matrix4f >  seg_frame_array1, seg_frame_array2;
	vector< Point3f >  joi_pos_array1, joi_pos_array2;
	Matrix4f  mat1, mat2;
	vector< vector< Vector3f > > v1(frames1, vector< Vector3f >(Num_segments));
	vector< vector< Vector3f > > v2(frames2, vector< Vector3f >(Num_segments));


	//初期化
	for(int i = 0; i < Num_segments; i++)
	{
		this->DTWorder[i] = i;
		this->DTWforder[i] = i;
		this->DisTotalPart[i] = 0.0f;
	}


	//順運動学計算
	for(int j = 0; j < frames1; j++)
	{
		ForwardKinematics( motion1.frames[ j ], seg_frame_array1, joi_pos_array1 );
		for(int k = 0; k < frames2; k++)
		{
			ForwardKinematics( motion2.frames[ k ], seg_frame_array2, joi_pos_array2 );
			for(int i = 0; i < Num_segments; i++)
			{
				//手の部位は計算しない
				while(i > 16 && i < 36)
					i++;
				if(i > 39)
					break;
				// 体節の中心の位置・向きを基準とする変換行列を適用
				mat1.transpose( seg_frame_array1[ i ] );
				v1[j][i] = {mat1.m30, mat1.m31, mat1.m32};
				mat2.transpose( seg_frame_array2[ i ] );
				v2[k][i] = {mat2.m30, mat2.m31, mat2.m32};
			}
		}
	}


	//距離の計算
	for(int j = 0; j <= frames1; j++)
	{
		for(int k = 0; k <= frames2; k++)
		{
			for(int i = 0; i < Num_segments; i++)
			{
				//手の部位は計算しない
				while (i > 16 && i < 36)
					i++;
				if (i > 39)
					break;

				if(j == frames1 || k == frames2)
				{
					this->DistanceAll[j][k] =100.0f;
					this->DistancePart[i][j][k] = 100.0f;
					break;
				}
				else
				{
					this->DistancePart[i][j][k] = sqrt(pow(v1[j][i].x - v2[k][i].x, 2.0) + pow(v1[j][i].y - v2[k][i].y, 2.0) + pow(v1[j][i].z - v2[k][i].z, 2.0));
					this->DistanceAll[j][k] += this->DistancePart[i][j][k];
				}
			}
		}
	}
	

	//全体に対するパスの作成
	int f1 = 0, f2 = 0;
	this->PassAll.resize(2, vector<int>());
	this->PassAll[0].push_back(0);
	this->PassAll[1].push_back(0);
	float dis1, dis2, dis3;
	while(f1 != frames1 && f2 != frames2)
	{
		dis1 = this->DistanceAll[f1+1][f2+1] + 0.0005f * abs(f1-f2);
		dis2 = this->DistanceAll[f1+1][f2] + 0.0005f * abs(f1+1-f2);
		dis3 = this->DistanceAll[f1][f2+1] + 0.0005f * abs(f1-(f2+1));
		if(dis1 < dis2 && dis1 < dis3)
			if(dis2 < dis3)
			{
				this->PassAll[0].push_back(f1);
				this->PassAll[1].push_back(++f2);
			}
			else
			{
				this->PassAll[0].push_back(++f1);
				this->PassAll[1].push_back(f2);
			}
		else
		{
			this->PassAll[0].push_back(++f1);
			this->PassAll[1].push_back(++f2);
		}
	}
	this->PassAll[0].erase(this->PassAll[0].end() - 1);
	this->PassAll[1].erase(this->PassAll[1].end() - 1);
	this->DTWframe = PassAll[0].size();

	//部位の集合ごとの誤差
	for(int f = 0; f < this->DTWframe; f++)
	{
		this->head.push_back(this->DistancePart[11][PassAll[0][f]][PassAll[1][f]] + 
			this->DistancePart[12][PassAll[0][f]][PassAll[1][f]]);
		this->chest.push_back(this->DistancePart[0][PassAll[0][f]][PassAll[1][f]] + 
			this->DistancePart[7][PassAll[0][f]][PassAll[1][f]] + 
			this->DistancePart[8][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[9][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[10][PassAll[0][f]][PassAll[1][f]]);
		this->head_chest.push_back(this->head.back() + this->chest.back());

		this->right_leg.push_back(this->DistancePart[1][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[2][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[3][PassAll[0][f]][PassAll[1][f]]);
		this->left_leg.push_back(this->DistancePart[4][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[5][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[6][PassAll[0][f]][PassAll[1][f]]);
		this->leg.push_back(this->right_leg.back() + this->left_leg.back());

		this->right_arm.push_back(this->DistancePart[13][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[14][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[15][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[16][PassAll[0][f]][PassAll[1][f]]);
		this->left_arm.push_back(this->DistancePart[36][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[37][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[38][PassAll[0][f]][PassAll[1][f]] +
			this->DistancePart[39][PassAll[0][f]][PassAll[1][f]]);
		this->arm.push_back(this->right_arm.back() + this->left_arm.back());
	}

	//部位毎の誤差の合計値を取る
	for(int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;

		for(int f = 0; f < this->DTWframe; f++)
			this->DisTotalPart[i] += this->DistancePart[i][PassAll[0][f]][PassAll[1][f]];
	}

	//誤差の大きい順にDTWorderを並べ替える
	for(int i = 0; i < Num_segments - 1; i++)
		for(int j = i; j < Num_segments; j++)
			if(this->DisTotalPart[i] < this->DisTotalPart[j])
				std::swap(this->DTWorder[i], this->DTWorder[j]);

	//誤差の合計を加算
	for(int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;

		//全部位の誤差の合計を計算
		this->DisTotalAll += this->DisTotalPart[i];

		std::cout << " [ " << this->DTWorder[i] << " segment: " << this->DisTotalPart[this->DTWorder[i]] << " ] ";
	}
	std::cout << std::endl;
}

//
//DTWフレーム毎の誤差
//
void DTWinformation::DTWinformation_frame(int now_frame, int num_segments)
{
	for(int i = 0; i < num_segments - 1; i++)
	{
		for(int j = i; j < num_segments; j++)
			if(this->DistancePart[this->DTWforder[i]][now_frame] < this->DistancePart[this->DTWforder[j]][now_frame])
				std::swap(this->DTWforder[i], this->DTWforder[j]);
	}
}


