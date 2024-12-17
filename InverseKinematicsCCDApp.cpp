/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算（CCD法）アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "InverseKinematicsCCDApp.h"
#include "BVH.h"

// プロトタイプ宣言

// 順運動学計算（※レポート課題）
void  MyForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );



//
//  コンストラクタ
//
InverseKinematicsCCDApp::InverseKinematicsCCDApp()
{
	app_name = "Inverse Kinematics (CCD)";

	curr_posture = NULL;
	base_joint_no = -1;
	ee_joint_no = -1;

	draw_joints = true;
}


//
//  デストラクタ
//
InverseKinematicsCCDApp::~InverseKinematicsCCDApp()
{
	if ( curr_posture && curr_posture->body )
		delete  curr_posture->body;
	if ( curr_posture )
		delete  curr_posture;
}


//
//  初期化
//
void  InverseKinematicsCCDApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// 骨格モデルの初期化に使用する BVHファイル
	const char *  file_name = "sample_walking1.bvh";

	// 動作データを読み込み
	BVH *  bvh = new BVH( file_name );

	// BVH動作から骨格モデルを生成
	if ( bvh->IsLoadSuccess() )
	{
		Skeleton *  new_body = CoustructBVHSkeleton( bvh );

		// 姿勢の初期化
		if ( new_body )
		{
			curr_posture = new Posture( new_body );
			InitPosture( *curr_posture, new_body );
		}
	}

	// 動作データを削除
	delete  bvh;
}


//
//  開始・リセット
//
void  InverseKinematicsCCDApp::Start()
{
	GLUTBaseApp::Start();

	if ( !curr_posture )
		return;

	// 姿勢初期化
	InitPosture( *curr_posture );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );

	// 支点・末端関節の初期化
	base_joint_no = -1;
	ee_joint_no = -1;
}


//
//  画面描画
//
void  InverseKinematicsCCDApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// 視点が更新されたら関節点の位置を更新
	if ( curr_posture && is_view_updated )
	{
		UpdateJointPositions( *curr_posture );

		// 視点の更新フラグをクリア
		is_view_updated = false;
	}

	// 関節点を描画
	if ( curr_posture && draw_joints )
	{
		DrawJoint();
	}

	// 現在のモードを表示
	DrawTextInformation( 0, "Inverse Kinematics (CCD-IK)" );
}


//
//  マウスクリック
//
void  InverseKinematicsCCDApp::MouseClick( int button, int state, int mx, int my )
{
	GLUTBaseApp::MouseClick( button, state, mx, my );

	// 左ボタンが押されたら、IKの支点・末端関節を選択
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
	{
		// Shiftキーが押されていれば、支点関節を選択
		if ( glutGetModifiers() & GLUT_ACTIVE_SHIFT )
			SelectJoint( mx, my, false );
		// Shiftキーが押されていなければ、末端関節を選択
		else
			SelectJoint( mx, my, true );
	}
}


//
//  マウスドラッグ
//
void  InverseKinematicsCCDApp::MouseDrag( int mx, int my )
{
	// 左ボタンのドラッグ中は、IKの末端関節の目標位置を操作
	if ( drag_mouse_l )
	{
		MoveJoint( mx - last_mouse_x, my - last_mouse_y );
	}

	GLUTBaseApp::MouseDrag( mx, my );
}


//
//  キーボードのキー押下
//
void  InverseKinematicsCCDApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// v キーで関節点の描画の有無を変更
	if ( key == 'v' )
		draw_joints = !draw_joints;

	// r キーで姿勢をリセット
	if ( key == 'r' )
		Start();
}


//
//  末端関節から支点関節へのパス（関節の配列と各関節における末端関節の方向）を探索
// （支点関節の番号が -1 の場合は、ルート体節を支点とする）
// （joint_path_signs は、各関節の子側に末端関節がある場合は 1、親側に末端関節がある場合は -1 を出力）
//
void  FindJointPath( const Skeleton * body, int base_joint_no, int ee_joint_no, vector< int > & joint_path, vector< int > & joint_path_signs )
{
	// 出力の配列をクリア
	joint_path.clear();
	joint_path_signs.clear();

	// 探索時の現在の関節・体節
	const Joint *  joint = NULL;
	const Segment *  segment = NULL;

	// 末端関節から探索を開始
	joint = body->joints[ ee_joint_no ];

	// 末端関節からルート体節に向かうパスを探索
	while ( true )
	{
		// ルート側の隣の関節を辿り、ルートに到達したら終了
		segment = joint->segments[ 0 ];
		if ( segment->index == 0 )
			break;
		joint = segment->joints[ 0 ];

		// 現在の関節をパスに追加
		joint_path.push_back( joint->index );
		
		// 途中で支点関節に到達したら終了
		if ( joint->index == base_joint_no )
			break;
	}

	// 各関節における末端関節の方向を表す符号の配列を生成（全て子側に末端関節がある）
	joint_path_signs.resize( joint_path.size(), 1 );


	// 支点が常にルート体節、もしくは、末端関節とルート体節の間にあると仮定すれば、ここで終了しても構わない
	// それ以外の場所に支点関節がある場合は、ルートから支点関節までのパスを求めて追加する処理が必要となる
	return;

/*
	// ※レポート課題

	// 支点がルート体節 or 支点関節がルート体節から末端関節のパス上にある場合は、終了
	if ( ( base_joint_no == -1 ) || ( joint->index == base_joint_no ) )
		return;

	// 支点関節からルート体節へ向かうパス
	vector< int >  joint_path2;

	// 探索処理の終了判定用フラグ
	bool  termination = false;

	// 支点関節から探索を開始
	joint = body->joints[ base_joint_no ];

	// 支点関節からルート体節に向かうパスを探索
	while ( true )
	{
		// 末端からルートまでのパスと合流したかどうかを判定し、合流したら終了
		for ( ??? )
		{
			if ( ??? )
			{
				// 末端からルートまでのパスを、合流した体節の前の関節まで縮小
				joint_path.resize( i + 1 );
				termination = true;
				break;
			}
		}
		if ( termination )
			break;

		// 現在の関節をパスに追加
		joint_path2.push_back( joint->index );

		// ルート側の隣の関節を辿り、ルートに到達したら終了
		segment = joint->segments[ 0 ];
		if ( segment->index == 0 )
			break;
		joint = segment->joints[ 0 ];

		// 末端関節に到達した場合（ルートと支点関節の間に末端関節がある場合）は、
		// 末端関節からルートまでのパスはクリアして、支点関節から末端関節までのパスを使用
		if ( joint->index == ee_joint_no )
		{
			joint_path.clear();
			joint_path_signs.clear();
			break;
		}
	}

	// 末端からルートに向かうパスと、支点からルートに向かうパスを結合（後者は逆の順番で結合）
	// 各関節における末端関節の方向を表す符号の配列を生成
	joint_path_signs.resize( joint_path.size(), 1 );
	for ( int i = 0; i < joint_path2.size(); i++ )
	{
		joint_path.push_back( joint_path2[ ??? ] );
		joint_path_signs.push_back( -1 );
	}
*/

}


//
//  Inverse Kinematics 計算（CCD法）
//  入出力姿勢、支点関節番号（-1の場合はルートを支点とする）、末端関節番号、末端関節の目標位置を指定
//
void  InverseKinematicsCCDApp::ApplyInverseKinematics( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position )
{
	ApplyInverseKinematicsCCD( posture, base_joint_no, ee_joint_no, ee_joint_position );
}


//
//  Inverse Kinematics 計算（CCD法）
//  入出力姿勢、支点関節番号（-1の場合はルートを支点とする）、末端関節番号、末端関節の目標位置を指定
//
void  ApplyInverseKinematicsCCD( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position )
{
	// 最大繰り返し数の設定
	const int  max_iteration = 10;

	// 位置が収束したと判断するための閾値の設定
	const float  distance_threshold = 0.01f;


	// 順運動学計算結果の格納用変数
	vector< Matrix4f >  segment_frames;
	vector< Point3f >   joint_positions;

	// 骨格情報
	const Skeleton *  body = posture.body;

	// 現在の関節
	const Joint *  joint = NULL;

	// 現在の関節の支点側の体節
	const Segment *  segment = NULL;

	// ルートから見た現在の関節の方向（末端側の場合は 1、支点側の場合は -1）
	float  direction;

	// 末端関節の現在位置（ワールド座標系）
	Point3f  ee_pos;

	// 現在の関節の局所座標系（ワールド座標系における関節の位置＋親側の体節の向き）
	Matrix4f  local_frame;

	// ワールド座標系から現在の関節の局所座標系への変換行列
	Matrix4f  trans_mat;

	// 末端関節の現在位置（局所座標系）
	Point3f  local_pos;

	// 支点関節から末端関節へのベクトル（局所座標系）
	Vector3f  ee_vec;

	// 末端関節の現在位置から目標位置へのベクトル（局所座標系）
	Vector3f  goal_vec;

	// 関節の回転軸と回転角度
	Vector3f  rot_axis;
	float  rot_angle = 0.0f;

	// 現在の関節の回転
	Matrix3f  rot;

	// 末端関節の目標位置と現在位置の距離
	Vector3f  vec;
	float  dist = -1.0f;


	// 引数チェック
	if ( !posture.body || ( ee_joint_no == -1 ) || ( base_joint_no == ee_joint_no ) )
		return;

	// 末端関節から支点関節へのパス（関節の配列と各関節における末端関節の方向）を探索
	vector< int >  joint_path, joint_path_signs;
	FindJointPath( body, base_joint_no, ee_joint_no, joint_path, joint_path_signs );

	// 現在の姿勢での各体節・関節の位置・向きを計算（順運動学計算）
	ForwardKinematics( posture, segment_frames, joint_positions );

	// CCD法の繰り返し計算（末端関節の位置が収束するか、一定回数繰り返したら終了する）
	for ( int i = 0; i < max_iteration; i++ )
	{
		// 末端関節から支点関節に向かって順番に繰り返し
		for ( int j = 0; j < joint_path.size(); j++ )
		{
			// 現在の関節と支点側の体節を取得
			joint = body->joints[ joint_path[ j ] ];
			direction = (float) joint_path_signs[ j ];
			segment = ( direction > 0.0f ) ? joint->segments[ 0 ] : joint->segments[ 1 ];

			// 末端関節の現在位置を取得
			ee_pos = joint_positions[ ee_joint_no ];

			// ※ レポート課題

			// 現在の関節のローカル座標系を取得
//			mat = ???;

			// ワールド座標系から現在の関節のローカル座標系への変換行列を計算
//			inv_mat = ???;

			// 現在の関節から末端関節への方向ベクトル（現在の関節のローカル座標系）を計算
//			ee_vec = ???;

			// 現在の関節から目標位置への方向ベクトル（現在の関節のローカル座標系）を計算
//			goal_vec = ???;

			// 現在の関節の回転軸・回転角度（0～π）を計算
//			rot_axis = ???;
//			rot_angle = ???;

			// 回転角度が微少であれば、回転は適用せずにスキップする
			if ( rot_angle < 0.001f )
				continue;

			// 回転を適用（回転の方向を考慮しない）
//			rot.set( AxisAngle4f( rot_axis, rot_angle ) );
//			???;

			// 回転後の回転を設定
			posture.joint_rotations[ joint->index ].set( rot );

			// 末端関節と現在の関節の間にルート体節がある場合は、ルート体節に移動・回転を適用
			if ( direction < 0.0f )
			{
			}

			// 更新された姿勢にもとづいて、各体節・関節の位置・向きを再計算（順運動学計算）
			MyForwardKinematics( posture, segment_frames, joint_positions );
		}

		// 収束判定、末端関節の目標位置と現在位置の距離が閾値以下になったら終了
		ee_pos = joint_positions[ ee_joint_no ];
		vec.sub( ee_joint_position, ee_pos );
		dist = vec.lengthSquared();
		if ( dist < distance_threshold * distance_threshold )
			break;
	}
}


//
//  以下、補助処理
//


//
//  関節点の位置の更新
//
void  InverseKinematicsCCDApp::UpdateJointPositions( const Posture & posture )
{
	if ( !curr_posture )
		return;

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array, joint_world_positions );

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	double  projection_matrix[ 16 ];
	int  viewport_param[ 4 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );
	glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix );
	glGetIntegerv( GL_VIEWPORT, viewport_param );

	// 画面上の各関節点の位置を計算
	int  num_joints = joint_world_positions.size();
	GLdouble  spx, spy, spz;
	joint_screen_positions.resize( num_joints );
	for ( int i=0; i<num_joints; i++ )
	{
		const Point3f &  wp = joint_world_positions[ i ];
		Point3f &  sp = joint_screen_positions[ i ];

		gluProject( wp.x, wp.y, wp.z,
			model_view_matrix, projection_matrix, viewport_param,
			&spx, &spy, &spz );
		sp.x = spx;
		sp.y = viewport_param[ 3 ] - spy;
	}
}


//
//  関節点の描画
//
void  InverseKinematicsCCDApp::DrawJoint()
{
	if ( !curr_posture )
		return;

	// デプステストを無効にして、前面に上書きする
	glDisable( GL_DEPTH_TEST );

	// 関節点を描画（球を描画）
	for ( int i = 0; i < joint_world_positions.size(); i++ )
	{
		// 支点関節は赤で描画
		if ( i == base_joint_no )
			glColor3f( 1.0f, 0.0f, 0.0f );
		// 末端関節は緑で描画
		else if ( i == ee_joint_no )
			glColor3f( 0.0f, 1.0f, 0.0f );
		// 他の関節は青で描画
		else
			glColor3f( 0.0f, 0.0f, 1.0f );

		// 関節位置に球を描画
		const Point3f &  pos = joint_world_positions[ i ];
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	// 支点関節が指定されていない場合は、ルート体節を支点とする（ルート体節の位置に球を描画）
	if ( base_joint_no == -1 )
	{
		// ルート体節位置に球を描画
		glColor3f( 1.0f, 0.0f, 0.0f );
		const Point3f &  pos = curr_posture->root_pos;
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	glEnable( GL_DEPTH_TEST );
}


//
//  関節点の選択
//
void  InverseKinematicsCCDApp::SelectJoint( int mouse_x, int mouse_y, bool ee_or_base )
{
	if ( !curr_posture )
		return;

	const float  distance_threthold = 20.0f;
	float  distance, min_distance = -1.0f;
	int  closesed_joint_no = -1;
	float  dx, dy;

	// 入力座標と最も近い位置にある関節を探索
	for ( int i = 0; i < joint_screen_positions.size(); i++ )
	{
		dx = joint_screen_positions[ i ].x - mouse_x;
		dy = joint_screen_positions[ i ].y - mouse_y;
		distance = sqrt( dx * dx + dy * dy );
		if ( ( i == 0 ) || ( distance <= min_distance ) )
		{
			min_distance = distance;
			closesed_joint_no = i;
		}
	}

	// 距離が閾値以下であれば選択
	if ( ee_or_base )
	{
		if ( min_distance < distance_threthold )
			ee_joint_no = closesed_joint_no;
		else
			ee_joint_no = -1;
	}
	else
	{
		if ( min_distance < distance_threthold )
			base_joint_no = closesed_joint_no;
		else
			base_joint_no = -1;
	}
}


//
//  関節点の移動（視線に垂直な平面上で上下左右に移動する）
//
void  InverseKinematicsCCDApp::MoveJoint( int mouse_dx, int mouse_dy )
{
	if ( !curr_posture )
		return;

	// 末端関節が選択されていなければ終了
	if ( ee_joint_no == -1 )
		return;

	// 画面上の移動量と３次元空間での移動量の比率
	const float  mouse_pos_scale = 0.01f;

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );

	Vector3f  vec;
	Point3f &  pos = joint_world_positions[ ee_joint_no ];

	// カメラ座標系のX軸方向に移動
	vec.set( model_view_matrix[ 0 ], model_view_matrix[ 4 ], model_view_matrix[ 8 ] );
	pos.scaleAdd( mouse_dx * mouse_pos_scale, vec, pos );

	// カメラ座標系のX軸方向に移動
	vec.set( model_view_matrix[ 1 ], model_view_matrix[ 5 ], model_view_matrix[ 9 ] );
	pos.scaleAdd( - mouse_dy * mouse_pos_scale, vec, pos );

	// Inverse Kinematics 計算を適用
	ApplyInverseKinematics( *curr_posture, base_joint_no, ee_joint_no, pos );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );
}



