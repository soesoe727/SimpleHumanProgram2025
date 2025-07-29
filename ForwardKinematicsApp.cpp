/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  順運動学計算アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "ForwardKinematicsApp.h"



//
//  コンストラクタ
//
ForwardKinematicsApp::ForwardKinematicsApp()
{
	app_name = "Forward Kinematics";
}


//
//  開始・リセット
//
void  ForwardKinematicsApp::Start()
{
	MotionPlaybackApp::Start();

	// 配列初期化
	segment_frames.resize( motion->body->num_segments );
	joint_positions.resize( motion->body->num_joints );

	ForwardKinematics( *curr_posture, segment_frames, joint_positions );
}


//
//  画面描画
//
void  ForwardKinematicsApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}


	// 関節・体節の位置・向きを描画

	const float  axis_length = 0.2f;
	float  line_width;
	Matrix4f  frame;

	// デプステストを無効にして、前面に上書きする
	glDisable( GL_DEPTH_TEST );

	//// 関節点を描画（球を描画）
	//for ( int i=0; i<joint_positions.size(); i++ )
	//{
	//	// 関節位置に球を描画
	//	const Point3f &  pos = joint_positions[ i ];
	//	glColor3f( 0.0f, 0.0f, 1.0f );
	//	glPushMatrix();
	//	glTranslatef( pos.x, pos.z, pos.z );
	//	glutSolidSphere( 0.025f, 16, 16 );
	//	glPopMatrix();
	//}

	//// 体節の座標系を描画（座標軸を描画）
	//glGetFloatv( GL_LINE_WIDTH, &line_width );
	//glLineWidth( 2.0f );
	//for ( int i=0; i<segment_frames.size(); i++ )
	//{
	//	glPushMatrix();
	//	frame.transpose( segment_frames[ i ] );
	//	glMultMatrixf( & frame.m00 );
	//	glBegin( GL_LINES );
	//		glColor3f( 1.0f, 0.0f, 0.0f );
	//		glVertex3f( 0.0f, 0.0f, 0.0f );
	//		glVertex3f( axis_length, 0.0f, 0.0f );
	//		glColor3f( 0.0f, 1.0f, 0.0f );
	//		glVertex3f( 0.0f, 0.0f, 0.0f );
	//		glVertex3f( 0.0f, axis_length, 0.0f );
	//		glColor3f( 0.0f, 0.0f, 1.0f );
	//		glVertex3f( 0.0f, 0.0f, 0.0f );
	//		glVertex3f( 0.0f, 0.0f, axis_length );
	//	glEnd();
	//	glPopMatrix();
	//}
	//glLineWidth( line_width );

	glEnable( GL_DEPTH_TEST );


	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Forward Kinematics" );
	char  message[64];
	if ( motion )
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
	else
		sprintf( message, "Press 'L' key to Load a BVH file" );
	DrawTextInformation( 1, message );
}


//
//  アニメーション処理
//
void  ForwardKinematicsApp::Animation( float delta )
{
	MotionPlaybackApp::Animation( delta );

	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	if ( !curr_posture )
		return;

	// 順運動学計算
	MyForwardKinematics( *curr_posture, segment_frames, joint_positions );
}


//
//  順運動学計算
//
void  MyForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array )
{
	// 配列初期化
	seg_frame_array.resize( posture.body->num_segments );
	joi_pos_array.resize( posture.body->num_joints );

	// ルート体節の位置・向きを設定
	seg_frame_array[ 0 ].set( posture.root_ori, posture.root_pos, 1.0f );

	// Forward Kinematics 計算のための反復計算（ルート体節から末端体節に向かって繰り返し計算）
	MyForwardKinematicsIteration( posture.body->segments[ 0 ], NULL, posture, &seg_frame_array.front(), &joi_pos_array.front() );
}


//
//  Forward Kinematics 計算のための反復計算（ルート体節から末端体節に向かって繰り返し再帰呼び出し）
//
void  MyForwardKinematicsIteration( 
	const Segment *  segment, const Segment * prev_segment, const Posture & posture, 
	Matrix4f * seg_frame_array, Point3f * joi_pos_array )
{
	// 骨格情報
	const Skeleton *  body = posture.body;

	// 次の関節・体節
	Joint *  next_joint;
	Segment *  next_segment;

	// 次の関節・体節の変換行列
	Matrix4f  frame;

	// 計算用のベクトル・行列
	Vector3f  pos;
	Matrix4f  mat;
	
	// 現在の体節に接続している各関節に対して繰り返し
	for ( int j = 0; j < segment->num_joints; j++ )
	{
		// 次の関節・次の体節を取得
		next_joint = segment->joints[ j ];
		if ( next_joint->segments[ 0 ] != segment )
			next_segment = next_joint->segments[ 0 ];
		else
			next_segment = next_joint->segments[ 1 ];

		// 前の体節側（ルート体節側）の関節はスキップ
		if ( next_segment == prev_segment )
			continue;

		// 現在の体節の変換行列を取得
		frame = seg_frame_array[ segment->index ];
		
		// ※ レポート課題

		// 現在の体節の座標系から、接続関節への座標系への平行移動をかける
//		???;

		// 次の関節の位置を設定
		if ( joi_pos_array )
			joi_pos_array[ next_joint->index ] = pos;

		// 関節の回転行列をかける
//		???;

		// 関節の座標系から、次の体節の座標系への平行移動をかける
//		???;

		// 次の体節の変換行列を設定
		if ( seg_frame_array )
			seg_frame_array[ next_segment->index ] = frame;

		// 次の体節に対して繰り返し（再帰呼び出し）
		MyForwardKinematicsIteration( next_segment, segment, posture, seg_frame_array, joi_pos_array );
	}
}



