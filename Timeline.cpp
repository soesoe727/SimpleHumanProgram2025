/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  タイムライン描画機能
**/

//aaa

// GLUT を使用
#include <GL/glut.h>

#include "Timeline.h"



// コンストラクタ
Timeline::Timeline()
{
	view_width = 100;
	view_height = 100;
	view_left = 0;
	view_top = 0;
	draw_background = true;
	background_color.set( 0.8f, 0.8f, 0.8f, 1.0f );
	track_height = 60;
	margin_height = 4;
	draw_margin = true;
	margin_color.set( 0.4f, 0.4f, 0.4f, 1.0f );
	num_tracks = 3;

	view_start_time = 0.0f;
	view_end_time = 10.0f;

	is_updated = true;

	UpdateTimeline();
}

// デストラクタ
Timeline::~Timeline()
{
}



//
//  タイムラインの設定
//


//
//  描画領域の設定
//
void  Timeline::SetViewArea( int left, int right, int top, int num_tracks, int track_height, int margin_hight )
{
	this->num_tracks = num_tracks;
	this->track_height = track_height;
	this->margin_height = margin_hight;

	view_width = right - left;
	if ( view_width < 60 )
		view_width = 60;
	view_height = ComputeViewHeight();
	if ( view_height < 60 )
		view_height = 60;
	view_left = left;
	view_top =top;

	is_updated = true;
}


//
//  描画領域の設定（画面の下部に配置）
//
void  Timeline::SetViewAreaBottom( int left_margin, int right_margin, int bottom_margin, int num_tracks, int track_height, int margin_hight )
{
	this->num_tracks = num_tracks;
	this->track_height = track_height;
	this->margin_height = margin_hight;

	GLint  viewport[ 4 ];
	glGetIntegerv( GL_VIEWPORT, viewport );
	int  screen_width = viewport[ 2 ];
	int  screen_height = viewport[ 3 ];

	view_width = screen_width - left_margin - right_margin;
	if ( view_width < 60 )
		view_width = 60;
	view_height = ComputeViewHeight();
	if ( view_height < 60 )
		view_height = 60;
	view_left = left_margin;
	view_top = screen_height - bottom_margin - view_height;

	is_updated = true;
}


//
//  描画時刻範囲の設定
//
void  Timeline::SetTimeRange( float start, float end )
{
	view_start_time = start;
	view_end_time = end;
	is_updated = true;
}


//
//  描画色の設定
//
void  Timeline::SetBackgroundColor( const Color4f & color )
{
	draw_background = true;
	background_color = color;
}


void  Timeline::SetMarginColor( const Color4f & color )
{
	draw_margin = true;
	margin_color = color;
}

void  Timeline::ClearColor()
{
	draw_background = false;
	draw_margin = false;
}



//
//  タイムラインに含まれる要素・縦線の設定
//


//
//  要素の追加
//
int  Timeline::AddElement( float start_time, float end_time, const Color4f & color, const char * text, int track_no )
{
	ElementInfo  element;
	element.is_enable = true;
	element.start_time = start_time;
	element.end_time = end_time;
	element.color = color;
	element.gradation = false;
	element.color2 = color;
	if ( text )
		element.text = text;
	element.track_no = track_no;
	if ( track_no == -1 )
		element.track_no = all_elements.size() < num_tracks ? all_elements.size() : num_tracks;
	element.parent_no = -1;
	element.height_top = 0.0f;
	element.height_bottom = 1.0f;

	element.draw_flag = false;
	element.draw_text_flag = false;

	all_elements.push_back( element );
	return  all_elements.size() - 1;
}


//
//  要素の削除
//
void  Timeline::DeleteElement( int no )
{
	vector< ElementInfo >::iterator  it = all_elements.begin();
	for ( int i = 0; i < no; i++ )
		it ++;
	if ( it != all_elements.end() )
		all_elements.erase( it );
}

void  Timeline::DeleteAllElements()
{
	all_elements.clear();
}


//
//  要素の情報更新
//
void  Timeline::SetElementEnable( int no, bool is_enable )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->is_enable = is_enable;
	is_updated = true;
}

void  Timeline::SetElementTime( int no, float start_time, float end_time )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->start_time = start_time;
	element->end_time = end_time;
	is_updated = true;
}

void  Timeline::SetElementColor( int no, const Color4f & color )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->color = color;
	element->gradation = false;
}

void  Timeline::SetElementColor( int no, const Color4f & color, const Color4f & color2 )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->color = color;
	element->color2 = color2;
	element->gradation = true;
}

void  Timeline::SetElementText( int no, const char * text )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	if ( strcmp( element->text.c_str(), text ) != 0 )
		element->text = text;
}

void  Timeline::SetElementTrackNo( int no, int track_no )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->track_no = track_no;
}

void  Timeline::SetElementParentNo( int no, int parent_no )
{
	ElementInfo *  element = GetElement( no );
	if ( !element )
		return;
	element->parent_no = parent_no;
}

//
//  サブ要素の追加
//
int  Timeline::AddSubElement( int parent, float start_time, float end_time, float top, float bottom, const Color4f & color )
{
	ElementInfo  element;
	element.is_enable = true;
	element.start_time = start_time;
	element.end_time = end_time;
	element.color = color;
	element.gradation = false;
	element.color2 = color;
	element.track_no = -1;
	element.parent_no = parent;
	element.height_top = top;
	element.height_bottom = bottom;

	element.draw_flag = false;
	element.draw_text_flag = false;

	all_sub_elements.push_back( element );
	return  all_sub_elements.size() - 1;
}


//
//  サブ要素の削除
//
void  Timeline::DeleteSubElement( int no )
{
	vector< ElementInfo >::iterator  it = all_sub_elements.begin();
	for ( int i = 0; i < no; i++ )
		it ++;
	if ( it != all_sub_elements.end() )
		all_sub_elements.erase( it );
}

void  Timeline::DeleteAllSubElements()
{
	all_sub_elements.clear();
}



//
//  サブ要素の情報更新
//
void  Timeline::SetSubElementEnable( int no, bool is_enable )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->is_enable = is_enable;
	is_updated = true;
}

void  Timeline::SetSubElementParent( int no, int parent )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->parent_no = parent;
}

void  Timeline::SetSubElementTime( int no, float start_time, float end_time )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->start_time = start_time;
	element->end_time = end_time;
	is_updated = true;
}

void  Timeline::SetSubElementHeight( int no, float top, float bottom )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->height_top = top;
	element->height_bottom = bottom;
	is_updated = true;
}

void  Timeline::SetSubElementColor( int no, const Color4f & color )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->color = color;
	element->gradation = false;
}

void  Timeline::SetSubElementColor( int no, const Color4f & color, const Color4f & color2 )
{
	ElementInfo *  element = GetSubElement( no );
	if ( !element )
		return;
	element->color = color;
	element->color2 = color2;
	element->gradation = true;
}


//
//  縦線の追加
//
int  Timeline::AddLine( float time, const Color4f & color, int start_track, int end_track )
{
	LineInfo  line;
	line.is_enable = true;
	line.time = time;
	line.color = color;
	line.start_track = start_track;
	line.end_track = end_track;

	all_lines.push_back( line );
	return  all_lines.size() - 1;
}


//
//  縦線の削除
//
void  Timeline::DeleteLine( int no )
{
	vector< LineInfo >::iterator  it = all_lines.begin();
	for ( int i = 0; i < no; i++ )
		it ++;
	if ( it != all_lines.end() )
		all_lines.erase( it );
}

void  Timeline::DeleteAllLines()
{
	all_lines.clear();
}


//
//  縦線の情報更新
//
void  Timeline::SetLineEnable( int no, bool is_enable )
{
	LineInfo *  line = GetLine( no );
	if ( !line )
		return;
	line->is_enable = is_enable;
	is_updated = true;
}

void  Timeline::SetLineTime( int no, float time )
{
	LineInfo *  line = GetLine( no );
	if ( !line )
		return;
	line->time = time;
	is_updated = true;
}

void  Timeline::SetLineColor( int no, const Color4f & color )
{
	LineInfo *  line = GetLine( no );
	if ( !line )
		return;
	line->color = color;
}


//
//  軌道の追加
//
int  Timeline::AddTrajectory( float start_time, float end_time, const Color4f & color, int num_values, const float * values, float value_min, float value_max, int track_no )
{
	TrajectoryInfo  tra;
	tra.is_enable = true;
	tra.start_time = start_time;
	tra.end_time = end_time;
	tra.color = color;
	if ( num_values > 0 )
	{
		tra.values.resize( num_values );
		for ( int i = 0; i < num_values; i++ )
			tra.values[ i ] = values[ i ];
	}
	tra.value_min = value_min;
	tra.value_max = value_max;
	tra.track_no = track_no;
	if ( track_no == -1 )
		tra.track_no = all_trajectories.size() < num_tracks ? all_trajectories.size() : num_tracks;

	tra.draw_flag = false;

	all_trajectories.push_back( tra );
	return  all_trajectories.size() - 1;
}

int  Timeline::AddTrajectory( float start_time, float end_time, const Color4f & color, vector< float > & values, float value_min, float value_max, int track_no )
{
	TrajectoryInfo  tra;
	tra.is_enable = true;
	tra.start_time = start_time;
	tra.end_time = end_time;
	tra.color = color;
	if ( values.size() > 0 )
		tra.values = values;
	tra.value_min = value_min;
	tra.value_max = value_max;
	tra.track_no = track_no;
	if ( track_no == -1 )
		tra.track_no = all_trajectories.size() < num_tracks ? all_trajectories.size() : num_tracks;

	tra.draw_flag = false;

	all_trajectories.push_back( tra );
	return  all_trajectories.size() - 1;
}


//
//  軌道の削除
//
void  Timeline::DeleteTrajectory( int no )
{
	vector< TrajectoryInfo >::iterator  it = all_trajectories.begin();
	for ( int i = 0; i < no; i++ )
		it ++;
	if ( it != all_trajectories.end() )
		all_trajectories.erase( it );
}

void  Timeline::DeleteAllTrajectories()
{
	all_trajectories.clear();
}


//
//  軌道の情報更新
//
void  Timeline::SetTrajectoryEnable( int no, bool is_enable )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->is_enable = true;
	is_updated = true;
}

void  Timeline::SetTrajectoryTime( int no, float start_time, float end_time )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->start_time = start_time;
	tra->end_time = end_time;
	is_updated = true;
}

void  Timeline::SetTrajectoryColor( int no, const Color4f & color )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->color = color;
	is_updated = true;
}

void  Timeline::SetTrajectoryValues( int no, int num_values, const float * values )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->values.resize( num_values );
	for ( int i = 0; i < num_values; i++ )
		tra->values[ i ] = values[ i ];
	is_updated = true;
}

void  Timeline::SetTrajectoryValues( int no, vector< float > & values )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->values = values;
	is_updated = true;
}

void  Timeline::SetTrajectoryValueMinMax( int no, float value_min, float value_max )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->value_min = value_min;
	tra->value_max = value_max;
	is_updated = true;
}

void  Timeline::SetTrajectoryValueMinMaxAuto( int no )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra || ( tra->values.size() == 0 ) )
		return;
	tra->value_min = tra->values[ 0 ];
	tra->value_max = tra->values[ 0 ];
	for ( int i = 1; i < tra->values.size(); i++ )
	{
		if ( tra->values[ i ] < tra->value_min )
			tra->value_min = tra->values[ i ];
		if ( tra->values[ i ] > tra->value_max )
			tra->value_max = tra->values[ i ];
	}
	is_updated = true;
}

void  Timeline::SetTrajectoryTrackNo( int no, int track_no )
{
	TrajectoryInfo *  tra = GetTrajectory( no );
	if ( !tra )
		return;
	tra->track_no = track_no;
	is_updated = true;
}


//
//  情報取得
//

int  Timeline::GetNumElements()
{
	return  all_elements.size();
}

int  Timeline::GetNumSubElements()
{
	return  all_sub_elements.size();
}

int  Timeline::GetNumLines()
{
	return  all_lines.size();
}

int  Timeline::GetNumTrajectories()
{
	return  all_trajectories.size();
}

Timeline::ElementInfo *  Timeline::GetElement( int no )
{
	if ( ( no < 0 ) || ( no >= all_elements.size() ) )
		return  NULL;
	return  &all_elements[ no ];
}

Timeline::ElementInfo *  Timeline::GetSubElement( int no )
{
	if ( ( no < 0 ) || ( no >= all_sub_elements.size() ) )
		return  NULL;
	return  &all_sub_elements[ no ];
}

Timeline::LineInfo *  Timeline::GetLine( int no )
{
	if ( ( no < 0 ) || ( no >= all_lines.size() ) )
		return  NULL;
	return  &all_lines[ no ];
}

Timeline::TrajectoryInfo *  Timeline::GetTrajectory( int no )
{
	if ( ( no < 0 ) || ( no >= all_trajectories.size() ) )
		return  NULL;
	return  &all_trajectories[ no ];
}



//
//  タイムライン描画
//
void  Timeline::DrawTimeline()
{
	int  y, w;

	// 全要素の描画位置を更新
	UpdateTimeline();

	// スクリーン画面描画モードの開始
	BeginScreenMode();

	// 背景を描画
	if ( draw_background )
	{
		DrawRectangle( view_left, view_top, view_left + view_width, view_top + view_height, background_color );
	}

	// トラック間の間隔を描画
	if ( draw_margin )
	{
		for ( int i = 0; i < num_tracks + 1; i++ )
		{
			y = view_top + ( track_height + margin_height ) * i;
			DrawRectangle( view_left, y, view_left + view_width, y + margin_height, margin_color );
		}
	}

	// 全要素を描画
	for ( int i = 0; i < all_elements.size(); i++ )
	{
		const ElementInfo &  element = all_elements[ i ];
		if ( element.draw_flag )
		{
			if ( element.gradation )
				DrawRectangle( element.screen_x0, element.screen_y0, element.screen_x1, element.screen_y1, element.color, element.color2 );
			else
				DrawRectangle( element.screen_x0, element.screen_y0, element.screen_x1, element.screen_y1, element.color );
;		}
	}

	// 全サブ要素を描画
	for ( int i = 0; i < all_sub_elements.size(); i++ )
	{
		const ElementInfo &  element = all_sub_elements[ i ];
		if ( element.draw_flag )
		{
			if ( element.gradation )
				DrawRectangle( element.screen_x0, element.screen_y0, element.screen_x1, element.screen_y1, element.color, element.color2 );
			else
				DrawRectangle( element.screen_x0, element.screen_y0, element.screen_x1, element.screen_y1, element.color );
;		}
	}

	// 全要素のテキストを描画
	for ( int i = 0; i < all_elements.size(); i++ )
	{
		const ElementInfo &  element = all_elements[ i ];
		if ( element.draw_flag && element.draw_text_flag )
		{
			DrawText( element.screen_text_x, element.screen_text_y, element.text.c_str() );
;		}
	}

	// 全ラインを描画
	for ( int i = 0; i < all_lines.size(); i++ )
	{
		const LineInfo &  line = all_lines[ i ];
		if ( line.draw_flag )
			DrawLine( line.screen_x, line.screen_y0, line.screen_y1, line.color );
	}

	// 全軌道を描画
	for ( int i = 0; i < all_trajectories.size(); i++ )
	{
		const TrajectoryInfo &  tra = all_trajectories[ i ];
		if ( tra.draw_flag && ( tra.values.size() > 2 ) )
			DrawTrajectory( tra.screen_x0, tra.screen_y0, tra.screen_x1, tra.screen_y1, 
				tra.values.size(), &tra.values.front(), tra.value_min, tra.value_max, tra.color );
	}

	// スクリーン画面描画モードの終了
	EndScreenMode();
}


//
//  画面位置に対応するトラック番号を取得（範囲外の場合は -1 を返す）
//
int  Timeline::GetTrackByPosition( int screen_x, int screen_y )
{
	if ( screen_x <= view_left )
		return  -1;
	if ( screen_x >= view_left + view_width )
		return  -1;
	if ( screen_y <= view_top )
		return  -1;
	if ( screen_y >= view_top + view_height )
		return  -1;

	int  track_no = ( screen_y - view_top ) / ( track_height + margin_height );

	return  track_no;
}


//
//  画面位置に対応する時刻を取得
//
float  Timeline::GetTimeByPosition( int screen_x )
{
	if ( screen_x <= view_left )
		return  view_start_time;
	if ( screen_x >= view_left + view_width )
		return  view_end_time;

	return  ( screen_x - view_left ) * ( view_end_time - view_start_time ) / view_width + view_start_time;
}



//
//  内部処理
//


//
//  全要素の描画位置を更新
//
void  Timeline::UpdateTimeline()
{
	if ( !is_updated )
		return;
	is_updated = true;

	float  view_duration = view_end_time - view_start_time;
	float  view_scale = view_width / view_duration;
	int  text_width = 0, text_height = 0;

	// 各要素の描画位置を更新
	for ( int i = 0; i < all_elements.size(); i++ )
	{
		ElementInfo &  element = all_elements[ i ];

		// 要素が無効であれば描画しない
		if ( !element.is_enable )
		{
			element.draw_flag = false;
			continue;
		}

		// 要素を描画する垂直位置を計算
		element.screen_y0 = view_top + margin_height * ( element.track_no + 1 ) + track_height * element.track_no;
		element.screen_y1 = view_top + margin_height * ( element.track_no + 1 ) + track_height * ( element.track_no + 1 );

		// 要素を描画する水平位置を計算
		element.screen_x0 = ( element.start_time - view_start_time ) * view_scale + view_left;
		element.screen_x1 = ( element.end_time - view_start_time ) * view_scale + view_left;

		// 水平位置が描画範囲に含まれない場合は描画しない
		if ( ( element.screen_x1 < view_left ) || ( element.screen_x0 > view_left + view_width ) )
		{
			element.draw_flag = false;
			continue;
		}
		element.draw_flag = true;

		// 水平位置が描画範囲を超えないように修正
		if ( element.screen_x0 < view_left )
			element.screen_x0 = view_left;
		if ( element.screen_x1 > view_left + view_width )
			element.screen_x1 = view_left + view_width;

		// テキストを描画する水平位置を計算
		if ( element.text.size() > 0 )
		{
			// テキストの描画幅・高さを取得
			text_width = GetTextWidth( element.text.c_str() );
			text_height = GetTextHeight( element.text.c_str() );

			// 要素内にテキストが収まらない場合は、描画しない
			if ( text_width > element.screen_x1 - element.screen_x0 )
			{
				element.draw_text_flag = false;
				continue;
			}
			element.draw_text_flag = true;

			// テキストの描画位置を計算
			element.screen_text_x = element.screen_x0 + ( element.screen_x1 - element.screen_x0 - text_width ) * 0.5f;
			element.screen_text_y = element.screen_y1 - ( element.screen_y1 - element.screen_y0 - text_height ) * 0.5f;
		}
		else
			element.draw_text_flag = false;
	}

	// 各サブ要素の描画位置を更新
	for ( int i = 0; i < all_sub_elements.size(); i++ )
	{
		ElementInfo &  element = all_sub_elements[ i ];
		ElementInfo *  parent = GetElement( element.parent_no );
		if ( !parent )
			continue;

		// 要素が無効であれば描画しない
		if ( !element.is_enable )
		{
			element.draw_flag = false;
			continue;
		}

		// 要素を描画する垂直位置を計算
		element.screen_y0 = parent->screen_y0 + ( parent->screen_y1 - parent->screen_y0 ) * element.height_top;
		element.screen_y1 = parent->screen_y0 + ( parent->screen_y1 - parent->screen_y0 ) * element.height_bottom;

		// 要素を描画する水平位置を計算
		element.screen_x0 = ( element.start_time - view_start_time ) * view_scale + view_left;
		element.screen_x1 = ( element.end_time - view_start_time ) * view_scale + view_left;

		// 水平位置が描画範囲に含まれない場合は描画しない
		if ( ( element.screen_x1 < view_left ) || ( element.screen_x0 > view_left + view_width ) )
		{
			element.draw_flag = false;
			continue;
		}
		element.draw_flag = true;

		// 水平位置が描画範囲を超えないように修正
		if ( element.screen_x0 < view_left )
			element.screen_x0 = view_left;
		if ( element.screen_x1 > view_left + view_width )
			element.screen_x1 = view_left + view_width;
	}

	// 各縦線の描画位置を更新
	for ( int i = 0; i < all_lines.size(); i++ )
	{
		LineInfo &  line = all_lines[ i ];

		// 縦線が無効であれば描画しない
		if ( !line.is_enable )
		{
			line.draw_flag = false;
			continue;
		}

		// 縦線を描画する垂直位置を計算
		if ( ( line.start_track >= 0 ) && ( line.end_track >= 0 ) )
		{
			line.screen_y0 = view_top + margin_height * ( line.start_track + 1 ) + track_height * ( line.start_track );
			line.screen_y1 = view_top + margin_height * ( line.end_track + 1 ) + track_height * ( line.end_track + 1 );
		}
		else
		{
			line.screen_y0 = view_top;
			line.screen_y1 = view_top + view_height;
		}

		// 縦線を描画する水平位置を計算
		line.screen_x = ( line.time - view_start_time ) * view_scale + view_left;

		// 水平位置が描画範囲に含まれない場合は描画しない
		if ( ( line.screen_x < view_left ) || ( line.screen_x > view_left + view_width ) )
			line.draw_flag = false;
		else
			line.draw_flag = true;
	}

	// 各軌道の描画位置を更新
	for ( int i = 0; i < all_trajectories.size(); i++ )
	{
		TrajectoryInfo &  traj = all_trajectories[ i ];

		// 軌道が無効であれば描画しない
		if ( !traj.is_enable )
		{
			traj.draw_flag = false;
			continue;
		}

		// 要素を描画する垂直位置を計算
		traj.screen_y0 = view_top + margin_height * ( traj.track_no + 1 ) + track_height * traj.track_no;
		traj.screen_y1 = view_top + margin_height * ( traj.track_no + 1 ) + track_height * ( traj.track_no + 1 );

		// 要素を描画する水平位置を計算
		traj.screen_x0 = ( traj.start_time - view_start_time ) * view_scale + view_left;
		traj.screen_x1 = ( traj.end_time - view_start_time ) * view_scale + view_left;

		// 水平位置が描画範囲に含まれない場合は描画しない
		if ( ( traj.screen_x1 < view_left ) || ( traj.screen_x0 > view_left + view_width ) )
		{
			traj.draw_flag = false;
			continue;
		}
		traj.draw_flag = true;

		// 水平位置が描画範囲を超えないように修正
		if ( traj.screen_x0 < view_left )
			traj.screen_x0 = view_left;
		if ( traj.screen_x1 > view_left + view_width )
			traj.screen_x1 = view_left + view_width;
	}
}


//
//  全体の高さを計算
//
int  Timeline::ComputeViewHeight()
{
	int  height = track_height * num_tracks + margin_height * ( num_tracks + 1 );
	return  height;
}


//
//  スクリーン描画モードの開始
//
void  Timeline::BeginScreenMode()
{
	GLint  viewport[ 4 ];
	glGetIntegerv( GL_VIEWPORT, viewport );
	int  screen_width = viewport[ 2 ];
	int  screen_height = viewport[ 3 ];

	// 射影行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, screen_width, screen_height, 0.0 );

	// モデルビュー行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	// OpenGLの描画設定を退避
	glPushAttrib( GL_ENABLE_BIT | GL_LINE_BIT );

	// Ｚバッファ・ライティングをオフにする
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	// ラインの太さを設定
	glLineWidth( 1.0f );
}


//
//  スクリーン描画モードの終了
//
void  Timeline::EndScreenMode()
{
	// OpenGLの描画設定を復元
	glPopAttrib();

	// 設定を全て復元
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}


//
//  四角形を描画
//
void  Timeline::DrawRectangle( int x0, int y0, int x1, int y1, const Color4f & color )
{
	glColor4f( color.x, color.y, color.z, color.w );
	glBegin( GL_QUADS );
		glVertex2f( x0, y0 );
		glVertex2f( x0, y1 );
		glVertex2f( x1, y1 );
		glVertex2f( x1, y0 );
	glEnd();
}

void  Timeline::DrawRectangle( int x0, int y0, int x1, int y1, const Color4f & color, const Color4f & color2 )
{
	glBegin( GL_QUADS );
		glColor4f( color.x, color.y, color.z, color.w );
		glVertex2f( x0, y0 );
		glVertex2f( x0, y1 );
		glColor4f( color2.x, color2.y, color2.z, color2.w );
		glVertex2f( x1, y1 );
		glVertex2f( x1, y0 );
	glEnd();
}


//
//  縦線を描画
//
void  Timeline::DrawLine( int x, int y0, int y1, const Color4f & color )
{
	glColor4f( color.x, color.y, color.z, color.w );
	glBegin( GL_LINES );
		glVertex2f( x, y0 );
		glVertex2f( x, y1 );
	glEnd();
}


//
//  軌道を描画
//
void  Timeline::DrawTrajectory( int x0, int y0, int x1, int y1, int num_values, const float * values, float value_min, float value_max, const Color4f & color )
{
	float  v = 0.0f, vx = 0.0f, vy = 0.0f;

	if ( num_values < 2 )
		return;

	glColor4f( color.x, color.y, color.z, color.w );
	glBegin( GL_LINE_STRIP );
	for ( int i = 0; i < num_values; i++ )
	{
		v = values[ i ];
		vx = i * ( x1 - x0 ) / ( num_values - 1 ) + x0;
		if ( v <= value_min )
			vy = y1;
		else if ( v >= value_max )
			vy = y0;
		else
			vy = ( y0 - y1 ) * ( v - value_min ) / ( value_max - value_min ) + y1;

		glVertex2f( vx, vy );
	}
	glEnd();
}


//
//  テキストを描画
//
void  Timeline::DrawText( int x, int y, const char * text )
{
	if ( !text || ( *text == '\0' ) )
		return;

	glColor4f( 0.0f, 0.0f, 0.0f, 1.0f );
	glRasterPos2i( x, y );
	for ( const char * c = text; (*c) != '\0'; c++ )
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_12, (const int)(*c) );
}


//
//  テキストの描画幅を取得
//
int  Timeline::GetTextWidth( const char * text )
{
	int  width = 0;
	for ( const char * c = text; (*c)!='\0'; c++ )
		width += glutBitmapWidth( GLUT_BITMAP_HELVETICA_12, (const int)(*c) );
	return  width;
}


//
//  テキストの描画高さを取得
//
int  Timeline::GetTextHeight( const char * text )
{
	return  12;
}



