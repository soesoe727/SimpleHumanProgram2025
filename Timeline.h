/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  タイムライン描画機能
**/

#ifndef  _TIMELINE_H_
#define  _TIMELINE_H_


#include <vecmath.h>

#include <vector>
using namespace  std;



//
//  タイムライン描画機能クラス
//
class  Timeline
{
  public:
	/*  データ構造の定義  */
	
	// 要素情報
	struct  ElementInfo
	{
		/*  設定情報  */

		// 有効フラグ
		bool              is_enable;

		// 時刻範囲（開始・終了時刻）
		float             start_time;
		float             end_time;

		// 描画色
		Color4f           color;

		// グラデーションで描画するかのフラグと描画色
		bool              gradation;
		Color4f           color2;

		// 表示テキスト
		string            text;

		// 描画するトラック番号
		int               track_no;

		// 親要素番号（サブ要素のために使用）
		int               parent_no;

		// 描画領域の高さの範囲（0.0～1.0）（サブ要素のために使用）
		float             height_top;
		float             height_bottom;

		/*  描画用情報  */

		// 要素の描画フラグ
		bool              draw_flag;

		// スクリーン上の要素の描画位置
		int               screen_x0;
		int               screen_x1;
		int               screen_y0;
		int               screen_y1;

		// テキストの描画フラグ
		bool              draw_text_flag;

		// スクリーン上のテキストの描画位置
		int               screen_text_x;
		int               screen_text_y;
	};

	// 縦線情報
	struct  LineInfo
	{
		/*  設定情報  */

		// 有効フラグ
		bool               is_enable;

		// 時刻
		float              time;

		// 描画色
		Color4f            color;

		// 描画するトラック番号の範囲
		int                start_track;
		int                end_track;

		/*  描画用情報  */

		// 縦棒の描画フラグ
		bool               draw_flag;

		// スクリーン上の縦棒の描画位置
		int                screen_x;
		int                screen_y0;
		int                screen_y1;
	};

	// 軌道情報
	struct  TrajectoryInfo
	{
		/*  設定情報  */

		// 有効フラグ
		bool              is_enable;

		// 時刻範囲（開始・終了時刻）
		float             start_time;
		float             end_time;

		// 描画色
		Color4f           color;

		// 軌道の値の配列
//		int               num_values;
//		float             values;
		vector< float >   values;

		// 軌道の値の範囲
		float             value_min;
		float             value_max;

		// 描画するトラック番号
		int               track_no;

		// 親要素番号（サブ要素のために使用）
//		int               parent_no;

		/*  描画用情報  */

		// 要素の描画フラグ
		bool              draw_flag;

		// スクリーン上の要素の描画位置
		int               screen_x0;
		int               screen_x1;
		int               screen_y0;
		int               screen_y1;
	};

  protected:
	/*  描画用の内部情報  */
	
	// タイムラインの描画領域
	int                    view_width;
	int                    view_height;
	int                    view_left;
	int                    view_top;

	// 描画トラック数
	int                    num_tracks;

	// 各トラックの高さ
	int                    track_height;

	// 背景描画の設定
	bool                   draw_background;
	Color4f                background_color;

	// ライン間の間隔の描画の設定
	int                    margin_height;
	bool                   draw_margin;
	Color4f                margin_color;

	// 描画時刻範囲
	float                  view_start_time;
	float                  view_end_time;

  protected:
	/*  タイムライン情報  */
	
	// 要素情報
	vector< ElementInfo >  all_elements;

	// サブ要素情報
	vector< ElementInfo >  all_sub_elements;

	// 縦線情報
	vector< LineInfo >     all_lines;

	// 軌道情報
	vector< TrajectoryInfo >  all_trajectories;

	// タイムライン情報の更新フラグ
	bool                   is_updated;

  public:
	// コンストラクタ
	Timeline();

	// デストラクタ
	virtual ~Timeline();

  public:
	// タイムラインの設定

	// 描画領域の設定（画面の任意の位置に配置）
	void  SetViewArea( int left, int right, int top, int num_tracks, int track_height, int margin_hight );

	// 描画領域の設定（画面の下部に配置）
	void  SetViewAreaBottom( int left_margin, int right_margin, int bottom_margin, int num_tracks, int track_height, int margin_hight );

	// 描画時刻範囲の設定
	void  SetTimeRange( float start, float end );

	// 描画色の設定
	void  SetBackgroundColor( const Color4f & color );
	void  SetMarginColor( const Color4f & color );
	void  ClearColor();

  public:
	// タイムラインに含まれる要素・縦線の設定

	// 要素の追加
	int  AddElement( float start_time, float end_time, const Color4f & color, const char * text = NULL, int track_no = -1 );

	// 要素の削除
	void  DeleteElement( int no );
	void  DeleteAllElements();

	// 要素の情報更新
	void  SetElementEnable( int no, bool is_enable );
	void  SetElementTime( int no, float start_time, float end_time );
	void  SetElementColor( int no, const Color4f & color );
	void  SetElementColor( int no, const Color4f & color, const Color4f & color2 );
	void  SetElementText( int no, const char * text );
	void  SetElementTrackNo( int no, int track_no );
	void SetElementParentNo(int no, int parent_no);

	// サブ要素の追加
	int  AddSubElement( int parent, float start_time, float end_time, float top, float bottom, const Color4f & color );

	// サブ要素の削除
	void  DeleteSubElement( int no );
	void  DeleteAllSubElements();

	// サブ要素の情報更新
	void  SetSubElementEnable( int no, bool is_enable );
	void  SetSubElementParent( int no, int parent );
	void  SetSubElementTime( int no, float start_time, float end_time );
	void  SetSubElementHeight( int no, float top, float bottom );
	void  SetSubElementColor( int no, const Color4f & color );
	void  SetSubElementColor( int no, const Color4f & color, const Color4f & color2 );

	// 縦線の追加
	int  AddLine( float time, const Color4f & color, int start_track = -1, int end_track = -1 );

	// 縦線の削除
	void  DeleteLine( int no );
	void  DeleteAllLines();

	// 縦線の情報更新
	void  SetLineEnable( int no, bool is_enable );
	void  SetLineTime( int no, float time );
	void  SetLineColor( int no, const Color4f & color );

	// 軌道の追加
	int  AddTrajectory( float start_time, float end_time, const Color4f & color, int num_values, const float * values, float value_min, float value_max, int track_no = -1 );
	int  AddTrajectory( float start_time, float end_time, const Color4f & color, vector< float > & values, float value_min, float value_max, int track_no = -1 );

	// 軌道の削除
	void  DeleteTrajectory( int no );
	void  DeleteAllTrajectories();

	// 軌道の情報更新
	void  SetTrajectoryEnable( int no, bool is_enable );
	void  SetTrajectoryTime( int no, float start_time, float end_time );
	void  SetTrajectoryColor( int no, const Color4f & color );
	void  SetTrajectoryValues( int no, int num_values, const float * values );
	void  SetTrajectoryValues( int no, vector< float > & values );
	void  SetTrajectoryValueMinMax( int no, float value_min, float value_max );
	void  SetTrajectoryValueMinMaxAuto( int no );
	void  SetTrajectoryTrackNo( int no, int track_no );

  public:
	// 情報取得
	int  GetViewHeight() { return  view_height; }
	int  GetNumTracks() { return  num_tracks; }
	int  GetNumElements();
	int  GetNumSubElements();
	int  GetNumLines();
	int  GetNumTrajectories();
	ElementInfo *  GetElement( int no );
	ElementInfo *  GetSubElement( int no );
	LineInfo *  GetLine( int no );
	TrajectoryInfo *  GetTrajectory( int no );

  public:
	// 描画処理

	// タイムライン描画
	virtual void  DrawTimeline();

	// 画面位置に対応するトラック番号を取得（範囲外の場合は -1 を返す）
	int  GetTrackByPosition( int screen_x, int screen_y );

	// 画面位置に対応する時刻を取得
	float  GetTimeByPosition( int screen_x );

  protected:
	// 内部処理

	// 全要素の描画位置を更新
	virtual void  UpdateTimeline();

	// 全体の高さを計算
	int  ComputeViewHeight();

	// スクリーン描画モードの開始・終了
	void  MotionPlaybackApp3();
	void  EndScreenMode();

	// 四角形を描画
	void  DrawRectangle( int x0, int y0, int x1, int y1, const Color4f & color );
	void  DrawRectangle( int x0, int y0, int x1, int y1, const Color4f & color, const Color4f & color2 );

	// 縦線を描画
	void  DrawLine( int x, int y0, int y1, const Color4f & color );

	// 軌道を描画
	void  DrawTrajectory( int x0, int y0, int x1, int y1, int num_values, const float * values, float value_min, float value_max, const Color4f & color );

	// テキストを描画
	void  DrawText( int x, int y, const char * text );

	// テキストの描画幅を取得
	int  Timeline::GetTextWidth( const char * text );

	// テキストの描画高さを取得
	int  Timeline::GetTextHeight( const char * text );
};


#endif // _TIMELINE_H_
