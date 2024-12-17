/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作接続・遷移
**/

#ifndef  _MOTION_TRANSITION_H_
#define  _MOTION_TRANSITION_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"


//
//  動作のメタ情報を表す構造体
//
struct  MotionInfo
{
	// 動作情報
	Motion *  motion;

	// 動作の開始・終了時刻（動作のローカル時間）
	float  begin_time;
	float  end_time;

	// 動作接続・遷移のためのブレンド区間の終了・開始時刻（動作のローカル時間）
	//（ begin_time <= blend_end_time < blend_begin_time <= end_time ）
	float  blend_end_time;
	float  blend_begin_time;

	// キー時刻の配列 [キーフレーム番号]
	vector< float >  keytimes;

	// 次の動作への動作接続のための基準部位
	int  base_segment_no;

	// 動作開始時・終了時（接続時）の水平向き
	bool  enable_ori;
	float  begin_ori;
	float  end_ori;

	// 描画色
	Color3f  color;
};


// 動作のメタ情報を初期化
void  InitMotionInfo( MotionInfo * info, Motion * m = NULL );


//
//  動作接続・遷移クラス
//
class  MotionTransition
{
  public:
	// 動作遷移の状態
	enum  MotionTransitionState
	{
		MT_NONE,           // 不明
		MT_PREV_MOTION,    // 前の動作の再生中
		MT_IN_TRANSITION,  // 前の動作から後の動作への遷移中
		MT_NEXT_MOTION     // 後の動作の再生中
	};

  protected:
	// 動作遷移の入力情報
	
	// 前の動作
	const MotionInfo *  prev_motion;

	// 後の動作
	const MotionInfo *  next_motion;

	// 前の動作の変換行列
	Matrix4f  prev_motion_mat;

	// 前の動作の開始時刻（グローバル時刻）
	float  prev_begin_time;

  protected:
	// 動作遷移のための情報
	
	// 動作接続のための基準体節番号
	int  base_segment_no;

	// 後の動作の位置・向きを前の動作の位置・向きに合わせるための変換行列
	Matrix4f  next_motion_mat;

	// 後の動作の開始時刻（前の動作の開始時刻を基準とするローカル時刻）
	float  next_begin_time;

	// 動作のブレンド開始時刻（前の動作の開始時刻を基準とするローカル時刻）
	float  blend_begin_time;

	// 動作のブレンド終了時刻（前の動作の開始時刻を基準とするローカル時刻）
	float  blend_end_time;

  protected:
	// 動作遷移中の姿勢補間のための情報

	// 前の動作の姿勢
	Posture *  prev_motion_posture;

	// 後の動作の姿勢
	Posture *  next_motion_posture;

	// 最後の姿勢計算時の動作遷移の状態
	MotionTransitionState  last_state;

	// 最後の姿勢計算時の前後の動作の姿勢補間の重み
	float  last_blend_ratio;

  public:
	// コンストラクタ
	MotionTransition();

	// デストラクタ
	virtual ~MotionTransition();

  public:
	// 情報取得
	const MotionInfo *  GetPrevMotion() const { return  prev_motion; }
	const MotionInfo *  GetNextMotion() const { return  next_motion; }
	const Matrix4f &  GetPrevMotionMatrix() const { return  prev_motion_mat; }
	const Matrix4f &  GetNextMotionMatrix() const { return  next_motion_mat; }
	int  GetBaseSegmentNo() const { return  base_segment_no; }
	float  GetPrevBeginTime() const { return  prev_begin_time; }
	float  GetLocalNextBeginTime() const { return  next_begin_time; }
	float  GetLocalBlendBeginTime() const { return  blend_begin_time; }
	float  GetLocalBlendEndTime() const { return  blend_end_time; }
	float  GetNextBeginTime() const { return  next_begin_time + prev_begin_time; }
	float  GetBlendBeginTime() const { return  blend_begin_time + prev_begin_time; }
	float  GetBlendEndTime() const { return  blend_end_time + prev_begin_time; }
	const Posture *  GetLastPrevPose() const { return  prev_motion_posture; }
	const Posture *  GetLastNextPose() const { return  next_motion_posture; }
	MotionTransitionState  GetLastState() const { return  last_state; }
	float  GetLastBlendRatio() const { return  last_blend_ratio; }
	
  public:
	// 動作接続・遷移
	
	// 動作接続・遷移の初期化
	virtual bool  Init( 
		const MotionInfo * prev_motion, const MotionInfo * next_motion, const Matrix4f & prev_motion_mat, float prev_begin_time );

	// 動作接続・遷移の姿勢計算
	virtual MotionTransitionState  GetPosture( float time, Posture * posture );
};


//
//  動作接続クラス（動作接続・遷移クラスの簡略版）
//
class  MotionConnection : public MotionTransition
{
  public:
	// コンストラクタ
	MotionConnection() : MotionTransition() {}

	// 動作接続の初期化
	virtual bool  Init( 
		const MotionInfo * prev_motion, const MotionInfo * next_motion, const Matrix4f & prev_motion_mat, float prev_begin_time );

	// 動作接続の姿勢計算
	virtual MotionTransitionState  GetPosture( float time, Posture * posture );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// ２つの姿勢の位置・向きを合わせるための変換行列を計算
//（next_frame の位置・向きを、prev_frame の位置・向きに合わせるための変換行列 trans_mat を計算）
void  ComputeConnectionTransformation( const Matrix4f & prev_frame, const Matrix4f & next_frame, Matrix4f & trans_mat );

// ２つの姿勢の位置・向きを合わせるための変換行列を計算
// ２つの姿勢の腰の水平位置・水平向きを合わせる
//（next_pose の腰の水平位置・水平向きを、prev_pose の腰の水平位置・水平向きに合わせる）
void  ComputeConnectionTransformation( const Posture & prev_pose, const Posture & next_pose, Matrix4f & trans_mat );

// ２つの姿勢の位置・向きを合わせるための変換行列を計算
// ２つの姿勢の基準部位の水平位置と、入力として与えられた水平向きを合わせる
//（next_pose の基準部位 base_segment の水平位置を prev_pose の水平位置に合わせて、水平向き next_ori を 水平向き prev_ori に合わせる）
void  ComputeConnectionTransformation( const Posture & prev_pose, float prev_ori, const Posture & next_pose, float next_ori, int base_segment, Matrix4f & trans_mat );



#endif // _MOTION_TRANSITION_H_
