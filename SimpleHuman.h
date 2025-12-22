/**
***  Simple Human Library and Samples for Interactive Character Animation
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/


#ifndef  _SIMPLE_HUMAN_H_
#define  _SIMPLE_HUMAN_H_


//
//  行列・ベクトルの表現には vecmath C++ライブラリ（http://objectclub.jp/download/vecmath1）を使用
//
#include <Vector3.h>
#include <Point3.h>
#include <Matrix3.h>
#include <Matrix4.h>
#include <Color3.h>
#include <Color4.h>

// STL（Standard Template Library）を使用
#include <vector>
#include <string>
using namespace  std;

// プロトタイプ宣言

struct  Segment;
struct  Joint;
class  Skeleton;
class  Posture;


//
//  人体モデルの体節を表す構造体
//
struct  Segment
{
	// 体節番号・名前
	int  index;
	string  name;

	// 体節の接続関節数
	int  num_joints;

	// 接続関節の配列 [接続関節番号（ルート体節以外は0番目の要素がルート側）]
	Joint **  joints;

	// 各接続関節の接続位置の配列（体節のローカル座標系）[接続関節番号（ルート体節以外は0番目の要素がルート側）]
	Point3f *  joint_positions;

	// 体節の末端位置
	bool  has_site;
	Point3f  site_position;
};


//
//  人体モデルの関節を表す構造体
//
struct  Joint
{
	// 関節番号・名前
	int  index;
	string  name;

	// 接続体節（0番目の要素がルート側、1番目の要素が末端側）
	Segment *  segments[ 2 ];
};


//
//  人体モデルの骨格を表すクラス
//
class  Skeleton
{
  public:
	// 体節数
	int  num_segments;

	// 体節の配列 [体節番号]
	Segment **  segments;

	// 関節数
	int  num_joints;

	// 関節の配列 [関節番号]
	Joint **  joints;


  public:
	// コンストラクタ・デストラクタ
	Skeleton();
	Skeleton( int s, int j );
	~Skeleton();
};


//
//  人体モデルの姿勢を表すクラス
//
class  Posture
{
  public:
	// 骨格モデル
	const Skeleton *  body;

	// ルートの位置
	Point3f  root_pos;

	// ルートの向き（回転行列表現）
	Matrix3f  root_ori;

	// 各関節の相対回転（回転行列表現）[関節番号]
	Matrix3f *  joint_rotations;


  public:
	// コンストラクタ・デストラクタ
	Posture();
	Posture( const Skeleton * b );
	Posture( const Posture & p );
	Posture &operator=( const Posture & p );
	~Posture();

	// 初期化
	void  Init( const Skeleton * b );

	// 順運動学計算（メンバ関数版）
	void  ForwardKinematics( vector< Matrix4f > & seg_frame_array ) const;
	void  ForwardKinematics( vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array ) const;
};


//
//  人体モデルの動作を表すクラス
//
class  Motion
{
  public:
	// 骨格モデル
	const Skeleton *  body;

	// フレーム数
	int  num_frames;

	// フレーム間の時間間隔
	float  interval;

	// 全フレームの姿勢 [フレーム番号]
	Posture *  frames;

	// 動作名
	string  name;


  public:
	// コンストラクタ・デストラクタ
	Motion();
	Motion( const Skeleton * b, int n );
	Motion( const Motion & m );
	Motion &operator=( const Motion & m );
	~Motion();

	// 初期化
	void  Init( const Skeleton * b, int n );

	// 動作の長さを取得
	float  GetDuration() const { return  num_frames * interval; }

	// 姿勢を取得
	Posture *  GetFrame( int no ) const;
	Posture *  GetFrameTime( float time ) const;
	void  GetPosture( float time, Posture & p ) const;
};



//
//  人体モデルのキーフレーム動作を表すクラス
//
class  KeyframeMotion
{
  public:
	// 骨格モデル
	const Skeleton *  body;

	// キーフレーム数
	int  num_keyframes;

	// 各キー時刻の配列 ［キーフレーム番号］ 
	float  *  key_times;

	// 各キー姿勢の配列 ［キーフレーム番号］ 
	Posture *  key_poses;


  public:
	// コンストラクタ・デストラクタ
	KeyframeMotion();
	KeyframeMotion( const Skeleton * b, int num );
	KeyframeMotion( const KeyframeMotion & m );
	KeyframeMotion &operator=( const KeyframeMotion & m );
	~KeyframeMotion();

	// 初期化
	void  Init( const Skeleton * b, int num );
	void  Init( const Skeleton * b, int num, const float * times, const Posture * poses );

	// 動作の長さを取得
	float  GetDuration() const;

	// 姿勢を取得
	float  GetKeyTime( int no ) const { return  key_times[ no ]; }
	Posture *  GetKeyPosture( int no ) const { return  & key_poses[ no ]; }
	void  GetPosture( float time, Posture & p ) const;
};


//
//  人体モデルの骨格・姿勢・動作の基本処理
//

// 姿勢の初期化（適当な腰の高さを計算・設定）
void  InitPosture( Posture & posture, const Skeleton * body = NULL );

// BVH動作から骨格モデルを生成
Skeleton *  CoustructBVHSkeleton( class BVH * bvh );

// BVH動作から動作データ（＋骨格モデル）を生成
Motion *  CoustructBVHMotion( class BVH * bvh, const Skeleton * bvh_body = NULL );

// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
Motion *  LoadAndCoustructBVHMotion( const char * bvh_file_name, const Skeleton * bvh_body = NULL );

// BVH動作から姿勢を取得
void  GetBVHPosture( const class BVH * bvh, int frame_no, Posture & posture );

// BVH動作の読み込み時の位置のスケールを取得
float  GetBVHScale();

// BVH動作の読み込み時の位置のスケールを設定
void  SetBVHScale( float scale );

// 骨格モデルから体節を名前で探索
int  FindSegment( const Skeleton * body, const char * segment_name );

// 骨格モデルから関節を名前で探索
int  FindJoint( const Skeleton * body, const char * joint_name );

// ADDED: 体節名から指かどうかを判定（BVHファイルによって体節数が変わる場合に対応）
bool  IsFingerSegment( const char * segment_name );
bool  IsFingerSegment( const Segment * segment );

// 順運動学計算
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );

// 順運動学計算
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array );

// 姿勢補間（２つの姿勢を補間）
void  PostureInterpolation( const Posture & p0, const Posture & p1, float ratio, Posture & p );

// 変換行列の水平向き（方位角）成分を計算（Ｚ軸の正の方向を０とする時計回りの角度を -180～180 の範囲で求める）
float  ComputeOrientationAngle( const Matrix3f & ori );
float  ComputeOrientationAngle( float dx, float dz );

// 水平回転を表す変換行列を計算（Ｚ軸の正の方向を０とする時計回りの角度を -180～180 の範囲で指定する）
void  ComputeOrientationMatrix( float angle, Matrix3f & ori );
Matrix3f  ComputeOrientationMatrix( float angle );

// 姿勢の位置・向きに変換行列を適用
void  TransformPosture( const Matrix4f & trans, Posture & posture );

// 骨格モデルの１本のリンクを楕円体で描画
void  DrawBone( float x0, float y0, float z0, float x1, float y1, float z1, float radius );

// 姿勢の描画（スティックフィギュアで描画）
void  DrawPosture( const Posture & posture );

// 姿勢の影の描画（スティックフィギュアで描画）
void  DrawPostureShadow( const Posture & posture, const Vector3f & light_dir, const Color4f & color );

// ADDED: 選択部位のみをハイライトして姿勢を描画
void  DrawPostureSelective( const Posture & posture, int selected_segment_index, const Color3f& highlight_color, const Color3f& base_color );


#endif // _SIMPLE_HUMAN_H_
