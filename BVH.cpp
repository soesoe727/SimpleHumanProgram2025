﻿/**
***  BVH動作ファイルの読み込み・描画クラス
***  Copyright (c) 2004-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/


#include <fstream>
#include <string.h>

#include "BVH.h"


// コントラクタ
BVH::BVH()
{
	motion = NULL;
	Clear();
}

// コントラクタ
BVH::BVH( const char * bvh_file_name )
{
	motion = NULL;
	Clear();

	Load( bvh_file_name );
}

// デストラクタ
BVH::~BVH()
{
	Clear();
}


// 全情報のクリア
void  BVH::Clear()
{
	int  i;
	for ( i=0; i<channels.size(); i++ )
		delete  channels[ i ];
	for ( i=0; i<joints.size(); i++ )
		delete  joints[ i ];
	if ( motion != NULL )
		delete  motion;

	is_load_success = false;
	
	file_name = "";
	motion_name = "";
	
	num_channel = 0;
	channels.clear();
	joints.clear();
	joint_index.clear();
	
	num_frame = 0;
	interval = 0.0;
	motion = NULL;
}


// 全情報を設定
void  BVH::Init( const char * name, 
	int n_joi, const Joint ** a_joi, int n_chan, const Channel ** a_chan, 
	int n_frame, double inter, const double * mo )
{
	// 関節の階層構造の設定
	SetSkeleton( name, n_joi, a_joi, n_chan, a_chan );

	// 動作データの設定
	SetMotion( n_frame, inter, mo );
}


// 関節の階層構造の設定
void  BVH::SetSkeleton( const char * name, 
	int n_joi, const Joint ** a_joi, int n_chan, const Channel ** a_chan )
{
	int  i, j;

	// 初期化
	Clear();

	// 基本情報の設定
	file_name = "";
	if ( name )
		motion_name = name;
	num_channel = n_chan;

	// チャンネル・関節配列の初期化
	channels.resize( num_channel );
	for ( i=0; i<num_channel; i++ )
		channels[ i ] = new Channel();
	joints.resize( n_joi );
	for ( i=0; i<n_joi; i++ )
		joints[ i ] = new Joint();

	// チャンネル・関節情報をコピー
	for ( i=0; i<num_channel; i++ )
	{
		*channels[ i ] = *a_chan[ i ];
		channels[ i ]->joint = joints[ a_chan[ i ]->joint->index ];
	}
	for ( i=0; i<n_joi; i++ )
	{
		*joints[ i ] = *a_joi[ i ];
		for ( j=0; j<joints[ i ]->channels.size(); j++ )
			joints[ i ]->channels[ j ] = channels[ a_joi[ i ]->channels[ j ]->index ];

		if ( a_joi[ i ]->parent == NULL )
			joints[ i ]->parent = NULL;
		else
			joints[ i ]->parent = joints[ a_joi[ i ]->parent->index ];

		for ( j=0; j<joints[ i ]->children.size(); j++ )
			joints[ i ]->children[ j ] = joints[ a_joi[ i ]->children[ j ]->index ];

		if ( joints[ i ]->name.size() > 0 )
			joint_index[ joints[ i ]->name ] = joints[ i ];
	}
}


// 動作データの設定
void  BVH::SetMotion( int n_frame, double inter, const double * mo )
{
	num_frame = n_frame;
	interval = inter;
	motion = new double[ num_frame * num_channel ];
	if ( mo != NULL )
		memcpy( motion, mo, sizeof( double ) * num_frame * num_channel );
}


//
//  BVHファイルのロード
//
void  BVH::Load( const char * bvh_file_name )
{
	#define  BUFFER_LENGTH  1024*4

	ifstream  file;
	char      line[ BUFFER_LENGTH ];
	char *    token;
	char      separater[] = " :,\t";
	vector< Joint * >   joint_stack;
	Joint *   joint = NULL;
	Joint *   new_joint = NULL;
	bool      is_site = false;
	double    x, y ,z;
	int       i, j;

	// 初期化
	Clear();

	// ファイルの情報（ファイル名・動作名）の設定
	file_name = bvh_file_name;
	const char *  mn_first = bvh_file_name;
	const char *  mn_last = bvh_file_name + strlen( bvh_file_name );
	if ( strrchr( bvh_file_name, '\\' ) != NULL )
		mn_first = strrchr( bvh_file_name, '\\' ) + 1;
	else if ( strrchr( bvh_file_name, '/' ) != NULL )
		mn_first = strrchr( bvh_file_name, '/' ) + 1;
	if ( strrchr( bvh_file_name, '.' ) != NULL )
		mn_last = strrchr( bvh_file_name, '.' );
	if ( mn_last < mn_first )
		mn_last = bvh_file_name + strlen( bvh_file_name );
	motion_name.assign( mn_first, mn_last );

	// ファイルのオープン
	file.open( bvh_file_name, ios::in );
	if ( file.is_open() == 0 )  return; // ファイルが開けなかったら終了

	// 階層情報の読み込み
	while ( ! file.eof() )
	{
		// ファイルの最後まできてしまったら異常終了
		if ( file.eof() )  goto bvh_error;

		// １行読み込み、先頭の単語を取得
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );

		// 空行の場合は次の行へ
		if ( token == NULL )  continue;

		// 関節ブロックの開始
		if ( strcmp( token, "{" ) == 0 )
		{
			// 現在の関節をスタックに積む
			joint_stack.push_back( joint );
			joint = new_joint;
			continue;
		}
		// 関節ブロックの終了
		if ( strcmp( token, "}" ) == 0 )
		{
			// 現在の関節をスタックから取り出す
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		// 関節情報の開始
		if ( ( strcmp( token, "ROOT" ) == 0 ) ||
		     ( strcmp( token, "JOINT" ) == 0 ) )
		{
			// 関節データの作成
			new_joint = new Joint();
			new_joint->index = joints.size();
			new_joint->parent = joint;
			new_joint->has_site = false;
			new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
			new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
			joints.push_back( new_joint );
			if ( joint )
				joint->children.push_back( new_joint );

			// 関節名の読み込み
			token = strtok( NULL, "" );
			while ( *token == ' ' )  token ++;
			new_joint->name = token;

			// インデックスへ追加
			joint_index[ new_joint->name ] = new_joint;
			continue;
		}

		// 末端情報の開始
		if ( ( strcmp( token, "End" ) == 0 ) )
		{
			new_joint = joint;
			is_site = true;
			continue;
		}

		// 関節のオフセット or 末端位置の情報
		if ( strcmp( token, "OFFSET" ) == 0 )
		{
			// 座標値を読み込み
			token = strtok( NULL, separater );
			x = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			y = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			z = token ? atof( token ) : 0.0;
			
			// 関節のオフセットに座標値を設定
			if ( is_site )
			{
				joint->has_site = true;
				joint->site[0] = x;
				joint->site[1] = y;
				joint->site[2] = z;
			}
			else
			// 末端位置に座標値を設定
			{
				joint->offset[0] = x;
				joint->offset[1] = y;
				joint->offset[2] = z;
			}
			continue;
		}

		// 関節のチャンネル情報
		if ( strcmp( token, "CHANNELS" ) == 0 )
		{
			// チャンネル数を読み込み
			token = strtok( NULL, separater );
			joint->channels.resize( token ? atoi( token ) : 0 );

			// チャンネル情報を読み込み
			for ( i=0; i<joint->channels.size(); i++ )
			{
				// チャンネルの作成
				Channel *  channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back( channel );
				joint->channels[ i ] = channel;

				// チャンネルの種類の判定
				token = strtok( NULL, separater );
				if ( strcmp( token, "Xrotation" ) == 0 )
					channel->type = X_ROTATION;
				else if ( strcmp( token, "Yrotation" ) == 0 )
					channel->type = Y_ROTATION;
				else if ( strcmp( token, "Zrotation" ) == 0 )
					channel->type = Z_ROTATION;
				else if ( strcmp( token, "Xposition" ) == 0 )
					channel->type = X_POSITION;
				else if ( strcmp( token, "Yposition" ) == 0 )
					channel->type = Y_POSITION;
				else if ( strcmp( token, "Zposition" ) == 0 )
					channel->type = Z_POSITION;
			}
		}

		// Motionデータのセクションへ移る
		if ( strcmp( token, "MOTION" ) == 0 )
			break;
	}


	// モーション情報の読み込み
	while ( ! file.eof() )
	{
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );
		if ( !token )
			continue;
		if ( strcmp( token, "Frames" ) == 0 )
			break;
	}
	if ( file.eof() )  goto bvh_error;
	token = strtok( NULL, separater );
	if ( token == NULL )  goto bvh_error;
	num_frame = atoi( token );

	while ( ! file.eof() )
	{
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, ":" );
		if ( !token )
			continue;
		if ( strcmp( token, "Frame Time" ) == 0 )
			break;
	}
	if ( file.eof() )  goto bvh_error;
	token = strtok( NULL, separater );
	if ( token == NULL )  goto bvh_error;
	interval = atof( token );

	num_channel = channels.size();
	motion = new double[ num_frame * num_channel ];

	// モーションデータの読み込み
	for ( i=0; i<num_frame; i++ )
	{
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );
		for ( j=0; j<num_channel; j++ )
		{
			if ( token == NULL )
				goto bvh_error;
			motion[ i*num_channel + j ] = atof( token );
			token = strtok( NULL, separater );
		}
	}

	// ファイルのクローズ
	file.close();

	// ロードの成功
	is_load_success = true;

	return;

bvh_error:
	file.close();
}


//
//  セーブ
//
void  BVH::Save( const char * bvh_file_name )
{
	int  i, j;
	ofstream  file;

	// モーションデータを出力するチャンネルの順番
	vector< int >   channel_order;

	// ファイルのオープン
	file.open( bvh_file_name, ios::out );
	if ( file.is_open() == 0 )  return; // ファイルが開けなかったら終了

	// 出力フォーマットの設定
	file.flags( ios::showpoint | ios::fixed );
	file.precision( 6 );
//	int  value_widht = 11;

	// 階層構造の出力
	file << "HIERARCHY" << endl;
	OutputHierarchy( file, joints[ 0 ], 0, channel_order );

	// モーションデータの出力
	file << "MOTION" << endl;
	file << "Frames: " << num_frame << endl;
	file << "Frame Time: " << interval << endl;
	for ( i=0; i<num_frame; i++ )
	{
		for ( j=0; j<channel_order.size(); j++ )
		{
			// 数値を固定幅で出力する場合は文字幅を設定
//			if ( value_widht > 0 )
//				file.width( value_widht );

			// モーションデータを出力
			file << GetMotion( i, channel_order[ j ] );

			// 空白か行末記号を出力
			if ( j != channel_order.size() - 1 )
				file << "  ";
			else
				file << endl;
		}
	}

	// ファイルのクローズ
	file.close();
}


//
//  セーブの補助関数
//

// 階層構造を再帰的に出力
void  BVH::OutputHierarchy( 
	ofstream & file, const Joint * joint, int indent_level, vector< int > & channel_list )
{
	int  i;
	string  indent, space;
	Channel *  channel;

	// インデント文字列の初期化
	indent.assign( indent_level * 4, ' ' );
	space.assign( "  " );

	// 関節名（ルート名）の出力
	if ( joint->parent )
		file << indent << "JOINT" << space << joint->name << endl;
	else
		file << indent << "ROOT" << space << joint->name << endl;

	// 関節ブロックの開始
	file << indent << "{" << endl;
	indent_level ++;
	indent.assign( indent_level * 4, ' ' );

	// オフセット位置の出力
	file << indent << "OFFSET" << space;
	file << joint->offset[0] << space;
	file << joint->offset[1] << space;
	file << joint->offset[2] << endl;

	// チャンネル情報の出力
	file << indent << "CHANNELS"  << space << joint->channels.size() << space;
	for ( i=0; i<joint->channels.size(); i++ )
	{
		channel = joint->channels[ i ];
		switch ( channel->type )
		{
		  case X_ROTATION:
			file << "Xrotation";  break;
		  case Y_ROTATION:
			file << "Yrotation";  break;
		  case Z_ROTATION:
			file << "Zrotation";  break;
		  case X_POSITION:
			file << "Xposition";  break;
		  case Y_POSITION:
			file << "Yposition";  break;
		  case Z_POSITION:
			file << "Zposition";  break;
		}
		if ( i != joint->channels.size() - 1 )
			file << space;
		else
			file << endl;

		// 出力チャンネルのリストに追加
		channel_list.push_back( channel->index );
	}

	// 末端位置の情報を出力
	if ( joint->has_site )
	{
		file << indent << "End Site" << endl;
		file << indent << "{" << endl;

		indent_level ++;
		indent.assign( indent_level * 4, ' ' );

		// オフセット位置の出力
		file << indent << "OFFSET" << space;
		file << joint->site[0] << space;
		file << joint->site[1] << space;
		file << joint->site[2] << endl;

		indent_level --;
		indent.assign( indent_level * 4, ' ' );

		file << indent << "}" << endl;
	}

	// 全ての子関節を再帰的に出力
	for ( i=0; i<joint->children.size(); i++ )
	{
		OutputHierarchy( file, joint->children[ i ], indent_level, channel_list );
	}

	// 関節ブロックの終了
	indent_level --;
	indent.assign( indent_level * 4, ' ' );
	file << indent << "}" << endl;
}


//
//  BVH骨格・姿勢の描画関数
//

#include <math.h>
#define  FREEGLUT_STATIC
#include <gl/glut.h>


// 指定フレームの姿勢を描画
void  BVH::RenderFigure( int frame_no, float scale )
{
	// BVH骨格・姿勢を指定して描画
	RenderFigure( joints[ 0 ], motion + frame_no * num_channel, scale );
}


// 指定されたBVH骨格・姿勢を描画（クラス関数）
void  BVH::RenderFigure( const Joint * joint, const double * data, float scale )
{
	glPushMatrix();

	// ルート関節の場合は平行移動を適用
	if ( joint->parent == NULL )
	{
		glTranslatef( data[ 0 ] * scale, data[ 1 ] * scale, data[ 2 ] * scale );
	}
	// 子関節の場合は親関節からの平行移動を適用
	else
	{
		glTranslatef( joint->offset[ 0 ] * scale, joint->offset[ 1 ] * scale, joint->offset[ 2 ] * scale );
	}

	// 親関節からの回転を適用（ルート関節の場合はワールド座標からの回転）
	int  i, j;
	for ( i=0; i<joint->channels.size(); i++ )
	{
		Channel *  channel = joint->channels[ i ];
		if ( channel->type == X_ROTATION )
			glRotatef( data[ channel->index ], 1.0f, 0.0f, 0.0f );
		else if ( channel->type == Y_ROTATION )
			glRotatef( data[ channel->index ], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == Z_ROTATION )
			glRotatef( data[ channel->index ], 0.0f, 0.0f, 1.0f );
	}

	// リンクを描画
	// 関節座標系の原点から末端点へのリンクを描画
	if ( joint->children.size() == 0 )
	{
		RenderBone( 0.0f, 0.0f, 0.0f, joint->site[ 0 ] * scale, joint->site[ 1 ] * scale, joint->site[ 2 ] * scale );
	}
	// 関節座標系の原点から次の関節への接続位置へのリンクを描画
	if ( joint->children.size() == 1 )
	{
		Joint *  child = joint->children[ 0 ];
		RenderBone( 0.0f, 0.0f, 0.0f, child->offset[ 0 ] * scale, child->offset[ 1 ] * scale, child->offset[ 2 ] * scale );
	}
	// 全関節への接続位置への中心点から各関節への接続位置へ円柱を描画
	if ( joint->children.size() > 1 )
	{
		// 原点と全関節への接続位置への中心点を計算
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( i=0; i<joint->children.size(); i++ )
		{
			Joint *  child = joint->children[ i ];
			center[ 0 ] += child->offset[ 0 ];
			center[ 1 ] += child->offset[ 1 ];
			center[ 2 ] += child->offset[ 2 ];
		}
		center[ 0 ] /= joint->children.size() + 1;
		center[ 1 ] /= joint->children.size() + 1;
		center[ 2 ] /= joint->children.size() + 1;

		// 原点から中心点へのリンクを描画
		RenderBone(	0.0f, 0.0f, 0.0f, center[ 0 ] * scale, center[ 1 ] * scale, center[ 2 ] * scale );

		// 中心点から次の関節への接続位置へのリンクを描画
		for ( i=0; i<joint->children.size(); i++ )
		{
			Joint *  child = joint->children[ i ];
			RenderBone(	center[ 0 ] * scale, center[ 1 ] * scale, center[ 2 ] * scale,
				child->offset[ 0 ] * scale, child->offset[ 1 ] * scale, child->offset[ 2 ] * scale );
		}
	}

	// 子関節に対して再帰呼び出し
	for ( i=0; i<joint->children.size(); i++ )
	{
		RenderFigure( joint->children[ i ], data, scale );
	}

	glPopMatrix();
}


// BVH骨格の１本のリンクを描画（クラス関数）
void  BVH::RenderBone( float x0, float y0, float z0, float x1, float y1, float z1 )
{
	// 与えられた２点を結ぶ円柱を描画

	// 円柱の２端点の情報を原点・向き・長さの情報に変換
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	// 描画パラメタの設定
	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	// 平行移動を設定
	glTranslated( x0, y0, z0 );

	// 以下、円柱の回転を表す行列を計算

	// ｚ軸を単位ベクトルに正規化
	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	// 基準とするｙ軸の向きを設定
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	// ｚ軸とｙ軸の外積からｘ軸の向きを計算
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	// ｘ軸を単位ベクトルに正規化
	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	// ｚ軸とｘ軸の外積からｙ軸の向きを計算
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	// 回転行列を設定
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	// 円柱の設定
	GLdouble radius= 0.01; // 円柱の太さ
	GLdouble slices = 8.0; // 円柱の放射状の細分数（デフォルト12）
	GLdouble stack = 3.0;  // 円柱の輪切りの細分数（デフォルト１）

	// 円柱を描画
	gluCylinder( quad_obj, radius, radius, bone_length, slices, stack ); 

	glPopMatrix();
}



// End of BVH.cpp
