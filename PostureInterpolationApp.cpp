/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  姿勢補間アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "PostureInterpolationApp.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  コンストラクタ
//
PostureInterpolationApp::PostureInterpolationApp()
{
	app_name = "Posture Interpolation";

	posture0 = NULL;
	posture1 = NULL;
	weight = 0.0f;

	curr_posture = NULL;
	figure_color.set( 1.0f, 1.0f, 1.0f );

	draw_fixed_position = true;
}


//
//  デストラクタ
//
PostureInterpolationApp::~PostureInterpolationApp()
{
	if ( curr_posture && curr_posture->body )
		delete  curr_posture->body;
	if ( posture0 )
		delete  posture0;
	if ( posture1 )
		delete  posture1;
	if ( curr_posture )
		delete  curr_posture;
}


//
//  初期化
//
void  PostureInterpolationApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// アプリケーションのテストに使用するサンプル姿勢（動作データ・時刻・描画色）
	const char *  sample_motion = "sample_walking1.bvh";
	const float  sample_keytimes[ 2 ] = { 3.00f, 3.74f };
	const Color3f  sample_colors[] = { Color3f( 0.5f, 1.0f, 0.5f ), Color3f( 0.5f, 0.5f, 1.0f ) };
	const float  sample_orientation[ 2 ] = { 180.0f, 180.0f };

	// 動作データの読み込み
	BVH  * bvh = new BVH( sample_motion );
	if ( !bvh->IsLoadSuccess() )
	{
		delete  bvh;
		return;
	}

	// BVH動作から骨格モデルを生成
	Skeleton *  body = CoustructBVHSkeleton( bvh );

	// サンプル姿勢・現在姿勢の生成
	posture0 = new Posture( body );
	posture1 = new Posture( body );
	curr_posture = new Posture( body );

	// サンプル姿勢を動作データから取得
	GetBVHPosture( bvh, sample_keytimes[ 0 ] / bvh->GetInterval(), *posture0 );
	GetBVHPosture( bvh, sample_keytimes[ 1 ] / bvh->GetInterval(), *posture1 );

	// 水平方向の回転が指定されていれば回転を適用
	Matrix3f  rot;
	if ( sample_orientation[ 0 ] != 0.0f )
	{
		rot.rotY( sample_orientation[ 0 ] * M_PI / 180.0f );
		posture0->root_ori.mul( rot, posture0->root_ori );
	}
	if ( sample_orientation[ 1 ] != 0.0f )
	{
		rot.rotY( sample_orientation[ 1 ] * M_PI / 180.0f );
		posture1->root_ori.mul( rot, posture1->root_ori );
	}

	// サンプル姿勢を描画する位置を設定（現在姿勢の左右に配置）
	posture0->root_pos.x = -1.0f;
	posture0->root_pos.z = 0.0f;
	posture1->root_pos.x = 1.0f;
	posture1->root_pos.z = 0.0f;

	// サンプル姿勢の描画色を設定
	posture0_color = sample_colors[ 0 ];
	posture1_color = sample_colors[ 1 ];

	// 姿勢補間の重みと現在姿勢を初期化
	weight = 0.0f;
	*curr_posture = *posture0;

	// 動作データの削除
	delete  bvh;
}


//
//  開始・リセット
//
void  PostureInterpolationApp::Start()
{
	GLUTBaseApp::Start();

	// 重みの初期化
	weight = 0.5f;

	// 姿勢更新
	UpdatePosture();
}


//
//  画面描画
//
void  PostureInterpolationApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( figure_color.x, figure_color.y, figure_color.z );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// サンプル姿勢を描画
	if ( posture0 )
	{
		glColor3f( posture0_color.x, posture0_color.y, posture0_color.z );
		DrawPosture( *posture0 );
		DrawPostureShadow( *posture0, shadow_dir, shadow_color );
	}
	if ( posture1 )
	{
		glColor3f( posture1_color.x, posture1_color.y, posture1_color.z );
		DrawPosture( *posture1 );
		DrawPostureShadow( *posture1, shadow_dir, shadow_color );
	}

	// 現在のモード、補間重みを表示
	DrawTextInformation( 0, "Posture Interpolation" );
	char  message[ 64 ];
	sprintf( message, "Weight: %.2f", weight );
	DrawTextInformation( 1, message );
}


//
//  マウスドラッグ
//
void  PostureInterpolationApp::MouseDrag( int mx, int my )
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
		UpdatePosture();
	}

	GLUTBaseApp::MouseDrag( mx, my );
}


//
//  キーボードのキー押下
//
void  PostureInterpolationApp::Keyboard( unsigned char key, int mx, int my )
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Keyboard( key, mx, my );

	// d キーで描画設定を変更
	if ( key == 'd' )
	{
		draw_fixed_position = !draw_fixed_position;
		UpdatePosture();
	}
}


//
//  姿勢更新
//
void  PostureInterpolationApp::UpdatePosture()
{
	if ( !curr_posture || !posture0 || !posture1 )
		return;

	// 姿勢補間
	MyPostureInterpolation( *posture0, *posture1, weight, *curr_posture );

	// 補間姿勢の腰の位置を固定して描画する場合は、腰の水平位置は原点に固定
	if ( draw_fixed_position )
	{
		curr_posture->root_pos.x = 0.0f;
		curr_posture->root_pos.z = 0.0f;
	}

	// 重みに応じて描画色を設定
	figure_color.scaleAdd( weight, posture1_color - posture0_color, posture0_color );
}


//
//  以下、補助処理
//


//
//  姿勢補間（２つの姿勢を補間）
//
void  MyPostureInterpolation( const Posture & p0, const Posture & p1, float ratio, Posture & p )
{
	// ２つの姿勢の骨格モデルが異なる場合は終了
	if ( ( p0.body != p1.body ) || ( p0.body != p.body ) )
		return;

	// 骨格モデルを取得
	const Skeleton *  body = p0.body;


	// ※ レポート課題

	// ２つの姿勢の各関節の回転を補間
	for ( int i = 0; i < body->num_joints; i++ )
	{
		// ???
	}

	// ２つの姿勢のルートの向きを補間
	// ???

	// ２つの姿勢のルートの位置を補間
	// ???
}



