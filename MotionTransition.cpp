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

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>


// 補助処理（グローバル関数）のプロトタイプ宣言

// 姿勢補間（２つの姿勢を補間）（※レポート課題）
void  MyPostureInterpolation( const Posture & p0, const Posture & p1, float ratio, Posture & p );



//
//  動作のメタ情報を初期化
//
void  InitMotionInfo( MotionInfo * info, Motion * m )
{
	info->motion = m;
	info->begin_time = 0.0f;
	info->end_time = info->motion ? info->motion->GetDuration() : 0.0f;
	info->blend_end_time = info->begin_time;
	info->blend_begin_time = info->end_time;
	info->base_segment_no = -1;
	info->enable_ori = false;
	info->begin_ori = 0.0f;
	info->end_ori = 0.0f;
	info->color.set( 1.0f, 1.0f, 1.0f );
}



//
//  動作接続・遷移クラス
//


// コンストラクタ
MotionTransition::MotionTransition()
{
	prev_motion = NULL;
	next_motion = NULL;
	prev_motion_posture = NULL;
	next_motion_posture = NULL;
	prev_motion_mat.setIdentity();
	next_motion_mat.setIdentity();
	last_state = MT_NONE;
}


// デストラクタ
MotionTransition::~MotionTransition()
{
	if ( prev_motion_posture )
		delete  prev_motion_posture;
	if ( next_motion_posture )
		delete  next_motion_posture;
}


//
//  動作接続・遷移の初期化
//
bool  MotionTransition::Init( 
	const MotionInfo * prev_motion, const MotionInfo * next_motion, const Matrix4f & prev_motion_mat, float prev_begin_time )
{
	// 入力チェック（骨格モデルが異なる動作間の動作接続・遷移には対応しない）
	if ( !prev_motion || !next_motion || ( prev_motion->motion->body != next_motion->motion->body ) )
		return  false;

	// 動作接続・遷移の入力情報を設定
	this->prev_motion = prev_motion;
	this->next_motion = next_motion;
	this->prev_motion_mat = prev_motion_mat;
	this->prev_begin_time = prev_begin_time;

	// 動作接続の基準部位を、前の動作の情報にもとづいて設定
	base_segment_no = prev_motion->base_segment_no;

	// 前後の動作から姿勢を取得するための変数を初期化
	if ( prev_motion_posture )
		delete  prev_motion_posture;
	if ( next_motion_posture )
		delete  next_motion_posture;
	prev_motion_posture = new Posture( prev_motion->motion->body );
	next_motion_posture = new Posture( next_motion->motion->body );

	// 動作遷移のタイミングの計算
	// 前の動作の blend_begin_time と後の動作の begin_time の時刻を合わせる

	// ※ レポート課題

	// 仮に動作接続の場合の時刻を設定（レポート課題作成前の動作確認用）
	next_begin_time = prev_motion->end_time - prev_motion->begin_time;
	blend_begin_time = next_begin_time;
	blend_end_time = next_begin_time;

	// 次の動作を開始する時刻（前の動作の開始時刻 prev_begin_time を基準とするローカル時刻）
//	next_begin_time = ???;

	// 動作遷移のための動作ブレンドを行う開始時刻（前の動作の開始時刻 prev_begin_time を基準とするローカル時刻）
//	blend_begin_time = ???;

	// 動作遷移のための動作ブレンドを行う終了時刻（前の動作の開始時刻 prev_begin_time を基準とするローカル時刻）
//	blend_end_time = ???;

	// 前後の動作の動作接続を行う時刻（各動作のローカル時刻）を取得
	float  prev_local_time = prev_motion->blend_begin_time;
	float  next_local_time = next_motion->begin_time;

	// 現在の動作の終了姿勢（ワールド座標系）を取得
	prev_motion->motion->GetPosture( prev_local_time, *prev_motion_posture );
	TransformPosture( prev_motion_mat, *prev_motion_posture );

	// 次の動作の開始姿勢（次の動作のワールド座標系）を取得
	next_motion->motion->GetPosture( next_local_time, *next_motion_posture );

	// 現在の動作の終了時の水平向き（ワールド座標系）、次の動作の開始時の水平向き（動作データのワールド座標系）
	float  prev_ori = 0.0f;
	float  next_ori = 0.0f;

	// 現在の動作データに向きの情報が設定されていれば、現在の変換行列と動作データの情報から終了時の水平向きを計算
	if ( prev_motion->enable_ori )
	{
		Matrix3f  prev_ori_mat;
		prev_motion_mat.get( &prev_ori_mat );
		prev_ori = ComputeOrientationAngle( prev_ori_mat );
		prev_ori += prev_motion->end_ori;
		if ( prev_ori > 180.0f )
			prev_ori -= 360.0f;
		if ( prev_ori < -180.0f)
			prev_ori += 360.0f;
	}
	// 前の動作の終了姿勢から水平向きを計算
	else
		prev_ori = ComputeOrientationAngle( prev_motion_posture->root_ori );

	// 次の動作データに向きの情報が設定されていれば、開始時の水平向きを取得
	if ( next_motion->enable_ori )
		next_ori = next_motion->begin_ori;
	// 次の動作の開始姿勢から水平向きを計算
	else
		next_ori = ComputeOrientationAngle( next_motion_posture->root_ori );

	// 現在の動作の終了姿勢と次の動作の開始姿勢の位置・向きを合わせるための変換行列を計算
	//（next_pose の基準部位 base_segment の水平位置を prev_pose の水平位置に合わせて、水平向き next_ori を 水平向き prev_ori に合わせる）
	ComputeConnectionTransformation( 
		*prev_motion_posture, prev_ori, *next_motion_posture, next_ori, base_segment_no, 
		next_motion_mat );

	return  true;
}


//
//  動作接続・遷移の姿勢計算
//
MotionTransition::MotionTransitionState  MotionTransition::GetPosture( 
	float time, Posture * posture )
{
	// 動作接続・遷移の初期化のチェック
	if ( !prev_motion || !next_motion )
		return  MT_NONE;

	// 前後の動作の姿勢補間の重み（ブレンド比率）
	float  blend_ratio = 0.0f;

	// 前の動作のローカル時刻（後の動作の開始時刻を基準とするローカル時刻）
	float  local_time = 0.0f;

	// 後の動作のローカル時刻（後の動作の開始時刻を基準とするローカル時刻）
	float  next_motion_local_time = 0.0f;

	// 前の動作の開始時刻を基準とするローカル時刻を計算
	local_time = time - prev_begin_time;

	// 現在の状態を判定
	MotionTransitionState  state = MT_PREV_MOTION;
	if ( local_time > blend_end_time )
		state = MT_NEXT_MOTION;
	else if ( local_time > blend_begin_time )
		state = MT_IN_TRANSITION;

	// 前の動作の姿勢を取得
	if ( state == MT_PREV_MOTION || state == MT_IN_TRANSITION )
	{
		// 前の動作の姿勢を取得
		prev_motion->motion->GetPosture( local_time + prev_motion->begin_time, *prev_motion_posture );

		// 前の動作の姿勢の位置・向きに変換行列を適用
		TransformPosture( prev_motion_mat, *prev_motion_posture );
	}

	// 後の動作の姿勢を取得
	if ( state == MT_NEXT_MOTION || state == MT_IN_TRANSITION )
	{
		// ※ レポート課題
		// 
		// 後の動作の現在時刻を計算（後の動作の開始時刻を基準とするローカル時刻）
//		next_motion_local_time = ???;

		// 後の動作から現在時刻の姿勢を取得
		next_motion->motion->GetPosture( next_motion_local_time + next_motion->begin_time, *next_motion_posture );

		// 後の動作の姿勢の位置・向きに変換行列を適用
		TransformPosture( next_motion_mat, *next_motion_posture );
	}

	// 動作遷移前であれば、前の動作の姿勢を出力
	if ( state == MT_PREV_MOTION )
	{
		blend_ratio = 0.0f;
		*posture = *prev_motion_posture;
	}
	// 動作遷移後であれば、後の動作の姿勢を出力
	else if ( state == MT_NEXT_MOTION )
	{
		blend_ratio = 1.0f;
		*posture = *next_motion_posture;
	}
	// 動作遷移中であれば、前後の動作の姿勢を補間
	else
	{
		// ※ レポート課題

		// ブレンド比率（補間の重み）を計算
//		blend_ratio = ???;

		// ブレンド比率（補間の重み）が範囲外の値になった場合への対応
		if ( blend_ratio > 1.0f )
			blend_ratio = 1.0f;
		if ( blend_ratio < 0.0f )
			blend_ratio = 0.0f;

		// 前後の動作の姿勢を補間
		MyPostureInterpolation( *prev_motion_posture, *next_motion_posture, blend_ratio, *posture );
	}

	// 最後の姿勢計算時の動作遷移の状態と姿勢補間の重みを記録
	last_state = state;
	last_blend_ratio = blend_ratio;

	// 動作遷移の状態を返す
	return  state;
}



//
//  動作接続クラス
//


//
//  動作接続の初期化
//
bool  MotionConnection::Init( 
	const MotionInfo * prev_motion, const MotionInfo * next_motion, const Matrix4f & prev_motion_mat, float prev_begin_time )
{
	// 入力チェック（骨格モデルが異なる動作間の動作接続・遷移には対応しない）
	if ( !prev_motion || !next_motion || ( prev_motion->motion->body != next_motion->motion->body ) )
		return  false;

	// 動作接続・遷移の入力情報を設定
	this->prev_motion = prev_motion;
	this->next_motion = next_motion;
	this->prev_motion_mat = prev_motion_mat;
	this->prev_begin_time = prev_begin_time;

	// 動作接続の基準部位を、前の動作の情報にもとづいて設定
	base_segment_no = prev_motion->base_segment_no;

	// 前後の動作から姿勢を取得するための変数を初期化
	if ( prev_motion_posture )
		delete  prev_motion_posture;
	if ( next_motion_posture )
		delete  next_motion_posture;
	prev_motion_posture = new Posture( prev_motion->motion->body );
	next_motion_posture = new Posture( next_motion->motion->body );

	// 動作遷移のタイミングの計算
	// 前の動作の end_time と後の動作の begin_time の時刻を合わせる
	// 動作遷移は行わないため、動作遷移の区間の長さは０にする

	// 後の動作を開始する時刻（前の動作の開始時刻 prev_begin_time を基準とするローカル時刻）
	next_begin_time = prev_motion->end_time - prev_motion->begin_time;

	// 動作遷移のための動作ブレンドの開始・終了時刻（前の動作の開始時刻 prev_begin_time を基準とするローカル時刻）
	blend_begin_time = next_begin_time;
	blend_end_time = next_begin_time;

	// 前後の動作の動作接続を行う時刻（各動作のローカル時刻）を決定
	float  prev_local_time = prev_motion->end_time;
	float  next_local_time = next_motion->begin_time;

	// 現在の動作の終了姿勢（ワールド座標系）を取得
	prev_motion->motion->GetPosture( prev_local_time, *prev_motion_posture );
	TransformPosture( prev_motion_mat, *prev_motion_posture );

	// 次の動作の開始姿勢（次の動作のワールド座標系）を取得
	next_motion->motion->GetPosture( next_local_time, *next_motion_posture );

	// 現在の動作の終了姿勢と次の動作の開始姿勢の姿勢の位置・向きを合わせるための変換行列を計算
	//（curr_posture の位置・向きを、next_posture の位置・向きに合わせるための変換行列 trans_mat を計算）
	ComputeConnectionTransformation( 
		*prev_motion_posture, 0.0f, *next_motion_posture, 180.0f, base_segment_no, 
		next_motion_mat );

	return  true;}


//
//  動作接続の姿勢計算
//
MotionTransition::MotionTransitionState  MotionConnection::GetPosture( float time, Posture * posture )
{
	// 動作接続・遷移の初期化のチェック
	if ( !prev_motion || !next_motion )
		return  MT_NONE;

	// 前後の動作の姿勢補間の重み（ブレンド比率）
	float  blend_ratio = 0.0f;

	// 前の動作のローカル時刻（後の動作の開始時刻を基準とするローカル時刻）
	float  local_time = 0.0f;

	// 後の動作のローカル時刻（後の動作の開始時刻を基準とするローカル時刻）
	float  next_motion_local_time = 0.0f;

	// 前の動作の開始時刻を基準とするローカル時刻を計算
	local_time = time - prev_begin_time;

	// 現在の状態を判定
	MotionTransitionState  state = MT_PREV_MOTION;
	if ( local_time > next_begin_time )
		state = MT_NEXT_MOTION;

	// 前の動作の姿勢を出力
	if ( state == MT_PREV_MOTION )
	{
		// 前の動作の姿勢を取得
		prev_motion->motion->GetPosture( local_time + prev_motion->begin_time, *posture );

		// 前の動作の姿勢の位置・向きに変換行列を適用
		TransformPosture( prev_motion_mat, *posture );

		// 姿勢補間の重みを設定
		blend_ratio = 0.0f;
	}

	// 後の動作の姿勢を出力
	else
	{
		// 後の動作の現在時刻を計算（後の動作の開始時刻を基準とするローカル時刻）
		next_motion_local_time = local_time - next_begin_time;

		// 後の動作から現在時刻の姿勢を取得
		next_motion->motion->GetPosture( next_motion_local_time + next_motion->begin_time, *posture );

		// 後の動作の姿勢の位置・向きに変換行列を適用
		TransformPosture( next_motion_mat, *posture );

		// 姿勢補間の重みを設定
		blend_ratio = 1.0f;
	}

	// 最後の姿勢計算時の動作遷移の状態と姿勢補間の重みを記録
	last_state = state;
	last_blend_ratio = blend_ratio;

	// 動作遷移の状態を返す
	return  state;
}



//
//  補助処理（グローバル関数）
//


//
//  ２つの姿勢の位置・向きを合わせるための変換行列を計算
// （next_frame の位置・向きを、prev_frame の位置・向きに合わせるための変換行列 trans_mat を計算）
//
void  ComputeConnectionTransformation( const Matrix4f & prev_frame, const Matrix4f & next_frame, Matrix4f & trans_mat )
{
	// ※ レポート課題

	// 方法１
	// trans_mat = ???
	
	// 方法２（水平方向の向き・位置のみを変換）
	Matrix3f  ori;
	float  angle;
	Vector3f  pos;
	Matrix4f  prev_frame2, next_frame2;

	// 変換行列から水平方向の回転のみを抽出して再設定
	// prev_frame2 = ???

	// 変換行列から水平方向の回転のみを抽出して再設定
	// next_frame2 = ???

	// 座標変換を計算
	// trans_mat = ???
}


//
//  ２つの姿勢の位置・向きを合わせるための変換行列を計算
//  ２つの姿勢の腰の水平位置・水平向きを合わせる
// （next_pose の腰の水平位置・水平向きを、prev_pose の腰の水平位置・水平向きに合わせる）
//
void  ComputeConnectionTransformation( const Posture & prev_pose, const Posture & next_pose, Matrix4f & trans_mat )
{
	// 前の姿勢の腰の位置・向きを取得
	Matrix4f  curr_end_frame;
	curr_end_frame.set( prev_pose.root_ori, prev_pose.root_pos, 1.0f );

	// 次の姿勢の腰の位置・向きを取得
	Matrix4f  next_begin_frame;
	next_begin_frame.set( next_pose.root_ori, next_pose.root_pos, 1.0f );

	// 次の姿勢の腰の位置・向きを前の姿勢の腰の位置・向きに合わせるための変換行列を計算
	ComputeConnectionTransformation( curr_end_frame, next_begin_frame, trans_mat );
}


//
//  ２つの姿勢の位置・向きを合わせるための変換行列を計算
//  ２つの姿勢の任意の体節の水平位置と、入力として与えられた水平向きを合わせる
// （next_pose の基準部位 base_segment の水平位置を prev_pose の水平位置に合わせて、水平向き next_ori を 水平向き prev_ori に合わせる）
//
void  ComputeConnectionTransformation( const Posture & prev_pose, float prev_ori, 
	const Posture & next_pose, float next_ori, int base_segment, Matrix4f & trans_mat )
{
/*
	Vector3f  prev_pos, next_pos;
	Matrix3f  prev_rot, next_rot;
	Matrix4f  prev_frame, next_frame;

	// 基準部位としてルート体節が指定された場合は、２つの姿勢のルートの位置を取得
	if ( base_segment == 0 )
	{
		// ※レポート課題
//		prev_pos = ???;
//		next_pos = ???;
	}
	// 基準部位としてルート体節以外が指定された場合は、順運動学計算により、２つの姿勢の指定部位の位置を計算
	else
	{
		// ※レポート課題
//		prev_pos = ???;
//		next_pos = ???;
	}

	// 前の姿勢の位置・向きを表す変換行列を計算
	// ※レポート課題
//	prev_rot = ???;
	prev_frame.set( prev_rot, prev_pos, 1.0f );

	// 次の姿勢の位置・向きを表す変換行列を計算
	// ※レポート課題
//	next_rot = ???;
	next_frame.set( next_rot, next_pos, 1.0f );

	// 座標変換を計算
	// ※レポート課題
//	trans_mat = ???;
*/

	// 上記の処理が未実装であれば、動作データの水平向きや基準部位の情報を使用しない、標準的な処理を呼び出す
	ComputeConnectionTransformation( prev_pose, next_pose, trans_mat );
}


