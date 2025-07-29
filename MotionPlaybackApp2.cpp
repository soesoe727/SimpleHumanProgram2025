/**
***  �L�����N�^�A�j���[�V�����̂��߂̐l�̃��f���̕\���E��{���� ���C�u�����E�T���v���v���O����
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  ����Đ��A�v���P�[�V����
**/

//test6
// ���C�u�����E�N���X��`�̓ǂݍ���
#include "SimpleHuman.h"
#include "BVH.h"
#include "MotionPlaybackApp.h"
#include "Timeline.h"
#define  _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iomanip>


//
//  �R���X�g���N�^
//
MotionPlaybackApp2::MotionPlaybackApp2()
{
	app_name = "Motion Playback2";

	motion = NULL;
	motion2 = NULL;
	curr_posture = NULL;
	curr_posture2 = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	DTWframe_no = 0;
	frame_no = 0;
	frames = 0;
	view_segment = 12;
	view_segments.resize(40,-1);
	gDTW = NULL;
	pattern = 0;
	timeline = NULL;
	show_color_matrix = true;
}


//
//  �f�X�g���N�^
//
MotionPlaybackApp2::~MotionPlaybackApp2()
{
	// ���i�E����E�p���̍폜
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// ���i�E����E�p���̍폜
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;

	if(gDTW)
		delete gDTW;
	if(timeline)
		delete timeline;
}


//
//  ������
//
void  MotionPlaybackApp2::Initialize()
{
	// ���N���X�̏��������s
	GLUTBaseApp::Initialize();
	gDTW = new DTWinformation2();
	// �T���v��BVH����f�[�^��ǂݍ���
	//LoadBVH( "sample_walking1.bvh" );
	OpenNewBVH();
	OpenNewBVH2();
	timeline = new Timeline();
	if(motion && motion2)
	{
		//���̈ʒu��0,z,0�ɂ��čŏ��Ɉꏏ�ɂ���
		Point3f root = {motion->frames[0].root_pos.x, 0.0f, motion->frames[0].root_pos.z}; 
		for(int i = 0; i < motion->num_frames; i++)
			motion->frames[i].root_pos -= root;
		Point3f root2 = motion2->frames[0].root_pos - motion->frames[0].root_pos;
		for(int i = 0; i < motion2->num_frames; i++)
			motion2->frames[i].root_pos -= root2;

		//std::cout << motion->frames[0].root_ori << std::endl;

		//Matrix3f p = motion->frames[0].root_ori, q;
		//Vector3f z_current = {motion->frames[0].root_ori.m02, motion->frames[0].root_ori.m12, motion->frames[0].root_ori.m22};
		//float theta = -atan2(z_current.x, z_current.z);
		//q.rotY(90);
		//for(int i = 0; i < motion->num_frames; i++)
		//	motion->frames[i].root_ori.mul(q, motion->frames[i].root_ori);
		
		//DTW�̍쐬
		gDTW->DTWinformation_init( motion->num_frames, motion2->num_frames, *motion, *motion2 );
	}
}

//
//  �J�n�E���Z�b�g
//
void  MotionPlaybackApp2::Start()
{
	// ���N���X�̏��������s
	GLUTBaseApp::Start();

	// �A�j���[�V�����Đ��̂��߂̕ϐ��̏�����
	on_animation = true;
	animation_time = 0.0f;
	DTWframe_no = 0;
	frame_no = 0;

	// �A�j���[�V�����Đ������i�p���̏������j
	Animation( 0.0f );

	//�J�����ʒu�̒���
	view_center = motion->frames[0].root_pos;
}


//
//  ��ʕ`��
//
void  MotionPlaybackApp2::Display()
{
	// ���N���X�̏��������s
	GLUTBaseApp::Display();
	Matrix4f mat;
	Point3f postureGravityPos1, postureGravityPos2;
	glLineWidth( 2.0f );

		// �^�C�����C����`��
	if ( timeline )
	{	
		//���ʂ̏W�����Ƃ̐F�t��
		if(sabun_flag == 1)//���[�s���O�E�p�X�ɂ��Đ�
			PatternTimeline(timeline, * motion, *motion2, DTWframe_no, gDTW);
		else//���[�s���O�E�p�X�ɂ��Đ�����ʏ�Đ�
			PatternTimeline(timeline, *motion, *motion2, frame_no, gDTW);
	}
	
	// �L�����N�^��`��
	if(curr_posture && curr_posture2)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		//�d�S�ʒu�덷�ɂ��Đ�
		if (sabun_flag == 1)//���[�s���O�E�p�X�ɂ��Đ�
		{
			//1�l�ڂ̓���̉���(�F�t��)
			Pass_posture1 = motion->frames[gDTW->warpingPath[0][DTWframe_no]];
			DrawPosture(Pass_posture1);
			DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
			

			//2�l�ڂ̓���̉���(����D)(�p�X�Ή�)
			Pass_posture2 = motion2->frames[gDTW->warpingPath[1][DTWframe_no]];
			DrawPosture(Pass_posture2);
			DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);

			postureGravityPos1 = gDTW->centerOfGravity1[gDTW->warpingPath[0][DTWframe_no]];
			postureGravityPos2 = gDTW->centerOfGravity2[gDTW->warpingPath[1][DTWframe_no]];
			glDisable( GL_DEPTH_TEST );
			glPushMatrix();
				glColor3f( 1.0f, 0.0f, 0.0f );
				glTranslatef( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
				glutSolidSphere( 0.025f, 16, 16 );
			glPopMatrix();
			glPushMatrix();
			glBegin( GL_LINES );
				glColor3f( 1.0f, 0.0f, 0.0f );
				glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
				glVertex3f(postureGravityPos1.x + 0.1f, postureGravityPos1.y, postureGravityPos1.z);
				glColor3f( 0.0f, 1.0f, 0.0f );
				glVertex3f( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
				glVertex3f(postureGravityPos1.x, postureGravityPos1.y + 0.1f, postureGravityPos1.z);
				glColor3f( 0.0f, 0.0f, 1.0f );
				glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
				glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z + 0.1f);
			glEnd();
			glPopMatrix();
			glPushMatrix();
			glColor3f( 0.0f, 1.0f, 0.0f );
			glTranslatef( postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z );
			glutSolidSphere( 0.025f, 16, 16 );
			glPopMatrix();
			glPushMatrix();
			glBegin( GL_LINES );
				glColor3f( 1.0f, 1.0f, 0.0f );
				glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
				glVertex3f(postureGravityPos2.x + 0.1f, postureGravityPos2.y, postureGravityPos2.z);
				glColor3f( 0.0f, 1.0f, 1.0f );
				glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
				glVertex3f(postureGravityPos2.x, postureGravityPos2.y + 0.1f, postureGravityPos2.z);
				glColor3f( 1.0f, 0.0f, 1.0f );
				glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
				glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z + 0.1f);
			glEnd();
			glPopMatrix();
			glEnable( GL_DEPTH_TEST );
		}
		else//���[�s���O�E�p�X�ɂ��Đ�����ʏ�Đ�
		{
			if (frame_no <= mf)//���[�s���O�E�p�X�ɂ��Đ�
			{
				//1�l�ڂ̓���̉���(�F�t��)
				Pass_posture1 = motion->frames[gDTW->warpingPath[0][frame_no]];
				DrawPosture(Pass_posture1);
				DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);

				//2�l�ڂ̓���̉���(����D)(�p�X�Ή�)
				Pass_posture2 = motion2->frames[gDTW->warpingPath[1][frame_no]];
				DrawPosture(Pass_posture2);
				DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);

				glDisable( GL_DEPTH_TEST );
				glPushMatrix();
				glColor3f( 1.0f, 0.0f, 0.0f );
				glTranslatef( gDTW->centerOfGravity1[gDTW->warpingPath[0][DTWframe_no]].x, gDTW->centerOfGravity1[gDTW->warpingPath[0][DTWframe_no]].y, gDTW->centerOfGravity1[gDTW->warpingPath[0][DTWframe_no]].z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glColor3f( 0.0f, 1.0f, 0.0f );
				glTranslatef( gDTW->centerOfGravity2[gDTW->warpingPath[1][DTWframe_no]].x, gDTW->centerOfGravity2[gDTW->warpingPath[1][DTWframe_no]].y, gDTW->centerOfGravity2[gDTW->warpingPath[1][DTWframe_no]].z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glEnable( GL_DEPTH_TEST );

				postureGravityPos1 = gDTW->centerOfGravity1[gDTW->warpingPath[0][frame_no]];
				postureGravityPos2 = gDTW->centerOfGravity2[gDTW->warpingPath[1][frame_no]];
				glDisable( GL_DEPTH_TEST );
				glPushMatrix();
					glColor3f( 1.0f, 0.0f, 0.0f );
					glTranslatef( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
					glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glBegin( GL_LINES );
					glColor3f( 1.0f, 0.0f, 0.0f );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
					glVertex3f(postureGravityPos1.x + 0.1f, postureGravityPos1.y, postureGravityPos1.z);
					glColor3f( 0.0f, 1.0f, 0.0f );
					glVertex3f( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y + 0.1f, postureGravityPos1.z);
					glColor3f( 0.0f, 0.0f, 1.0f );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z + 0.1f);
				glEnd();
				glPopMatrix();
				glPushMatrix();
				glColor3f( 0.0f, 1.0f, 0.0f );
				glTranslatef( postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glBegin( GL_LINES );
					glColor3f( 1.0f, 1.0f, 0.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x + 0.1f, postureGravityPos2.y, postureGravityPos2.z);
					glColor3f( 0.0f, 1.0f, 1.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y + 0.1f, postureGravityPos2.z);
					glColor3f( 1.0f, 0.0f, 1.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z + 0.1f);
				glEnd();
				glPopMatrix();
				glEnable( GL_DEPTH_TEST );
			}
			else//�ʏ�Đ�
			{
				//1�l�ڂ̓���̉���(�F�t��)
				Pass_posture1 = motion->frames[min(motion->num_frames - 1, m1f + frame_no - mf)];
				DrawPosture(Pass_posture1);
				DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
				//2�l�ڂ̓���̉���(����D)(�p�X�Ή�)
				Pass_posture2 = motion2->frames[min(motion2->num_frames - 1, m2f + frame_no - mf)];
				DrawPosture(Pass_posture2);
				DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);

				glDisable( GL_DEPTH_TEST );
				glPushMatrix();
				glColor3f( 1.0f, 0.0f, 0.0f );
				glTranslatef( gDTW->centerOfGravity1[min(motion->num_frames - 1, m1f + frame_no - mf)].x, gDTW->centerOfGravity1[min(motion->num_frames - 1, m1f + frame_no - mf)].y, gDTW->centerOfGravity1[min(motion->num_frames - 1, m1f + frame_no - mf)].z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glColor3f( 0.0f, 1.0f, 0.0f );
				glTranslatef( gDTW->centerOfGravity2[min(motion2->num_frames - 1, m2f + frame_no - mf)].x, gDTW->centerOfGravity2[min(motion2->num_frames - 1, m2f + frame_no - mf)].y, gDTW->centerOfGravity2[min(motion2->num_frames - 1, m2f + frame_no - mf)].z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glEnable( GL_DEPTH_TEST );

				postureGravityPos1 = gDTW->centerOfGravity1[min(motion->num_frames - 1, m1f + frame_no - mf)];
				postureGravityPos2 = gDTW->centerOfGravity2[min(motion2->num_frames - 1, m2f + frame_no - mf)];
				glDisable( GL_DEPTH_TEST );
				glPushMatrix();
					glColor3f( 1.0f, 0.0f, 0.0f );
					glTranslatef( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
					glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glBegin( GL_LINES );
					glColor3f( 1.0f, 0.0f, 0.0f );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
					glVertex3f(postureGravityPos1.x + 0.1f, postureGravityPos1.y, postureGravityPos1.z);
					glColor3f( 0.0f, 1.0f, 0.0f );
					glVertex3f( postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y + 0.1f, postureGravityPos1.z);
					glColor3f( 0.0f, 0.0f, 1.0f );
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z);
					glVertex3f(postureGravityPos1.x, postureGravityPos1.y, postureGravityPos1.z + 0.1f);
				glEnd();
				glPopMatrix();
				glPushMatrix();
				glColor3f( 0.0f, 1.0f, 0.0f );
				glTranslatef( postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z );
				glutSolidSphere( 0.025f, 16, 16 );
				glPopMatrix();
				glPushMatrix();
				glBegin( GL_LINES );
					glColor3f( 1.0f, 1.0f, 0.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x + 0.1f, postureGravityPos2.y, postureGravityPos2.z);
					glColor3f( 0.0f, 1.0f, 1.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y + 0.1f, postureGravityPos2.z);
					glColor3f( 1.0f, 0.0f, 1.0f );
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z);
					glVertex3f(postureGravityPos2.x, postureGravityPos2.y, postureGravityPos2.z + 0.1f);
				glEnd();
				glPopMatrix();
				glEnable( GL_DEPTH_TEST );
			}
		}
		glDisable(GL_BLEND);
	}

	// �^�C�����C����`��
	if ( show_color_matrix )
		timeline->DrawTimeline();

	// ���݂̃��[�h�A���ԁE�t���[���ԍ���\��
	DrawTextInformation( 0, "Motion Playback" );
	char  message1[64];

	if ( motion && motion2 )
	{
		if (sabun_flag == 1)
			sprintf(message1, "%.2f (%d)", animation_time, DTWframe_no);
		else
			sprintf(message1, "%.2f (%d)", animation_time, frame_no);

		DrawTextInformation( 1, message1 );
	}
	else
	{
		sprintf( message1, "Press 'L' key to Load BVH files" );
		DrawTextInformation( 1, message1 );
	}
	
}

//
//  �E�B���h�E�T�C�Y�ύX
//
void MotionPlaybackApp2::Reshape(int w, int h)
{
	GLUTBaseApp::Reshape( w, h );

	// �^�C�����C���̕`��̈�̐ݒ�i��ʂ̉����ɔz�u�j
	if ( timeline )
		timeline->SetViewAreaBottom(0, 1, 0, 10, 10, 2);
}

//
//  �L�[�{�[�h�̃L�[����
//
void  MotionPlaybackApp2::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// space �L�[�ŃA�j���[�V�����̒�~�E�ĊJ
	if ( key == ' ' )
		on_animation = !on_animation;

	// w �L�[�ŃA�j���[�V�����̍Đ����x��ύX
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

	//d �L�[��view_segment�̕ύX+
	if (key == 'd')
		view_segment = ViewSegment(1, view_segment);

	//a �L�[��view_segment�̕ύX-
	if (key == 'a')
		view_segment = ViewSegment(0, view_segment);

	// s �L�[��view_segments�̕ύX
	if (key == 's')
		view_segments[view_segment] *= -1;

	// x �L�[��view_segments�̑S�폜
	if (key == 'x')
		for_each(view_segments.begin(), view_segments.end(), [](int& n) {n = -1; });

	// c �L�[��view_segments�̑S����
	if (key == 'c')
		for_each(view_segments.begin(), view_segments.end(), [](int& n) {n = 1; });

	// 0 �L�[�Ńp�^�[��0
	if ( key == '0' )
		pattern = 0;
	// 1 �L�[�Ńp�^�[��1
	if ( key == '1' )
		pattern = 1;
	// 2 �L�[�Ńp�^�[��2
	if ( key == '2' )
		pattern = 2;
	// 3 �L�[�Ńp�^�[��3
	if ( key == '3' )
		pattern = 3;
	// 4 �L�[�Ńp�^�[��4
	if ( key == '4' )
		pattern = 4;
	// 5 �L�[�Ńp�^�[��5
	if ( key == '5' )
		pattern = 5;

	//f �L�[�ōĐ����@�؂�ւ�
	if(key == 'f')
	{
		if (sabun_flag == 1)
		{
			m1f = gDTW->warpingPath[0][DTWframe_no];
			m2f = gDTW->warpingPath[1][DTWframe_no];
			mf = DTWframe_no;
			frames = mf + motion->num_frames - m1f;
			frame_no = 0;
			std::cout << m1f << " " << m2f << " " << mf << std::endl;
			sabun_flag *= -1;
		}
		else
		{
			m1f = -30;
			m2f = -30;
			mf = -30;
			DTWframe_no = 0;
			sabun_flag *= -1;
		}
	}

	// m �L�[�ŃJ���[�}�g���b�N�X�̕\���؂�ւ�
	if (key == 'm')
	{
		if (show_color_matrix)
			show_color_matrix = false;
		else
			show_color_matrix = true;
	}
}

//
//  �}�E�X�h���b�O
//
void  MotionPlaybackApp2::MouseDrag( int mx, int my )
{
	GLUTBaseApp::MouseDrag( mx, my );
	int name_space = 30.0f;

	// �}�E�X���W�ɑΉ�����^�C�����C���̃g���b�N�ԍ��E�������擾
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if (selected_time < 0)
		selected_time = 0.0f;
	else if (sabun_flag == 1 && selected_time > gDTW->errorFrame - 1)
		selected_time = gDTW->errorFrame - 1;
	else if (sabun_flag == -1 && selected_time > frames - 1)
		selected_time = frames - 1;


	// �ό`����̍Đ�������ύX
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		drag_mouse_l = false;
		Animation( 0.0f );
		drag_mouse_l = true;
	}
}

//
//  �}�E�X�N���b�N
//
void MotionPlaybackApp2::MouseClick(int button, int state, int mx, int my)
{
	GLUTBaseApp::MouseClick( button, state, mx, my );
	int name_space = 30.0f;

	// �}�E�X���W�ɑΉ�����^�C�����C���̃g���b�N�ԍ��E�������擾
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if (selected_time < 0)
		selected_time = 0.0f;
	else if (sabun_flag == 1 && selected_time > gDTW->errorFrame - 1)
		selected_time = gDTW->errorFrame - 1;
	else if (sabun_flag == -1 && selected_time > frames - 1)
		selected_time = frames - 1;

	// ���͓���g���b�N��̃N���b�N�ʒu�ɉ����āA�ό`����̍Đ�������ύX
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		Animation( 0.0f );
	}
}

//
//  �A�j���[�V��������
//
void  MotionPlaybackApp2::Animation( float delta )
{
	// �A�j���[�V�����Đ����łȂ���ΏI��
	if ( !on_animation )
		return;

	// �}�E�X�h���b�O���̓A�j���[�V�������~
	if ( drag_mouse_l )
		return;

	// ����f�[�^���ǂݍ��܂�Ă��Ȃ���ΏI��
	if ( !motion || !motion2 )
		return;

	// ���Ԃ�i�߂�
	animation_time += delta * animation_speed;
	
	if( sabun_flag == 1)
	{
		if(error_flag == 1 && animation_time >= gDTW->errorFrame * motion->interval)
			animation_time = 0.0f;
		else if (error_flag == -1 && animation_time >= gDTW->AngFrame * motion->interval)
			animation_time = 0.0f;

		DTWframe_no = animation_time / motion->interval;
	}
	else if( sabun_flag == -1)
	{
		if(animation_time >= frames * motion->interval)
			animation_time = 0.0f;
		frame_no = animation_time / motion->interval;
	}
}



//
//  �⏕����
//

//
//view_segment�̏���
//
int  MotionPlaybackApp2::ViewSegment(int i, int seg)
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
//  BVH����t�@�C���̓ǂݍ��݁A����E�p���̏�����
//
void  MotionPlaybackApp2::LoadBVH( const char * file_name )
{
	// BVH�t�@�C����ǂݍ���œ���f�[�^�i�{���i���f���j�𐶐�
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );
	
	// BVH�t�@�C���̓ǂݍ��݂Ɏ��s������I��
	if ( !new_motion )
		return;
	
	// ���ݎg�p���Ă��鍜�i�E����E�p�����폜
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// ����Đ��Ɏg�p���铮��E�p����������
	motion = new_motion;
	curr_posture = new Posture( motion->body );

	// ����Đ��J�n
	//Start();
}
void  MotionPlaybackApp2::LoadBVH2( const char * file_name )
{
	// BVH�t�@�C����ǂݍ���œ���f�[�^�i�{���i���f���j�𐶐�
	Motion *  new_motion2 = LoadAndCoustructBVHMotion( file_name );
	
	// BVH�t�@�C���̓ǂݍ��݂Ɏ��s������I��
	if ( !new_motion2 )
		return;
	
	// ���ݎg�p���Ă��鍜�i�E����E�p�����폜
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;
	

	// ����Đ��Ɏg�p���铮��E�p����������
	motion2 = new_motion2;
	curr_posture2 = new Posture( motion2->body );
	
	// ����Đ��̊J�n
	//Start();
}

//
//  �t�@�C���_�C�A���O��\������BVH����t�@�C����I���E�ǂݍ���
//
void  MotionPlaybackApp2::OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// �t�@�C���_�C�A���O�̐ݒ�
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

	// �t�@�C���_�C�A���O��\��
	BOOL  ret = GetOpenFileName( &open_file );

	// �t�@�C�����w�肳�ꂽ��V���������ݒ�
	if ( ret )
	{
		// BVH����f�[�^�̓ǂݍ��݁A���i�E�p���̏�����
		LoadBVH( file_name );

		// ����Đ��̊J�n
		//Start();
	}
#endif // WIN32
}
void  MotionPlaybackApp2::OpenNewBVH2()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// �t�@�C���_�C�A���O�̐ݒ�
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

	// �t�@�C���_�C�A���O��\��
	BOOL  ret = GetOpenFileName( &open_file );

	// �t�@�C�����w�肳�ꂽ��V���������ݒ�
	if ( ret )
	{
		// BVH����f�[�^�̓ǂݍ��݁A���i�E�p���̏�����
		LoadBVH2( file_name );

		// ����Đ��̊J�n
		//Start();
	}
#endif // WIN32
}

//
//�^�C�����C���̃p�^�[�����ǂݍ���
//
void MotionPlaybackApp2::PatternTimeline(Timeline* timeline, Motion& motion, Motion& motion2, float curr_frame, DTWinformation2* DTW)
{
	int track_num = 0;
	float name_space = 30.0f;
	
	// �^�C�����C���̎��Ԕ͈͂�ݒ�
	timeline->SetTimeRange( 0.0f, DTW->errorFrame + num_space );
	// �S�v�f�E�c�_�̏����N���A
	timeline->DeleteAllElements();
	timeline->DeleteAllLines();
	timeline->DeleteAllSubElements();

	if (sabun_flag == 1)
	{
		timeline->AddElement( 0.0f, name_space, {1.0f,1.0f,1.0f,1.0f},  "Error", track_num );
		ColorBarElementErrorCenterOfGravity(timeline, track_num++, DTW->errorCenterOfGravity, DTW->warpingPath, motion, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {1.0f,0.0f,0.0f,1.0f},  "Center_X", track_num );
		ColorBarElementCenterOfGravity_X(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {1.0f,1.0f,0.0f,1.0f},  "Center_X", track_num );
		ColorBarElementCenterOfGravity_X(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {0.0f,1.0f,0.0f,1.0f},  "Center_Y", track_num );
		ColorBarElementCenterOfGravity_Y(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {0.0f,1.0f,1.0f,1.0f},  "Center_Y", track_num );
		ColorBarElementCenterOfGravity_Y(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {0.0f,0.0f,1.0f,1.0f},  "Center_Z", track_num );
		ColorBarElementCenterOfGravity_Z(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {1.0f,0.0f,1.0f,1.0f},  "Center_Z", track_num );
		ColorBarElementCenterOfGravity_Z(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
	}
	else
	{
		timeline->AddElement( 0.0f, name_space, {1.0f,1.0f,1.0f,0.2f},  "Error", track_num );
		ColorBarElementRepErrorCenterOfGravity(timeline, track_num++, DTW->errorCenterOfGravity, DTW->warpingPath, motion, motion2, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {1.0f,0.0f,0.0f,1.0f},  "Center_X", track_num );
		ColorBarElementRepCenterOfGravity_X(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {1.0f,1.0f,0.0f,1.0f},  "Center_X", track_num );
		ColorBarElementRepCenterOfGravity_X(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {0.0f,1.0f,0.0f,1.0f},  "Center_Y", track_num );
		ColorBarElementRepCenterOfGravity_Y(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {0.0f,1.0f,1.0f,1.0f},  "Center_Y", track_num );
		ColorBarElementRepCenterOfGravity_Y(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
		track_num++;
		timeline->AddElement( 0.0f, name_space, {0.0f,0.0f,1.0f,1.0f},  "Center_Z", track_num );
		ColorBarElementRepCenterOfGravity_Z(timeline, track_num++, DTW->centerOfGravity1, DTW->warpingPath[0], motion, curr_frame);
		timeline->AddElement( 0.0f, name_space, {1.0f,0.0f,1.0f,1.0f},  "Center_Z", track_num );
		ColorBarElementRepCenterOfGravity_Z(timeline, track_num++, DTW->centerOfGravity2, DTW->warpingPath[1], motion2, curr_frame);
	}

	// ����Đ�������\���c����ݒ�
	timeline->AddLine( curr_frame + num_space, Color4f( 1.0f, 1.0f, 1.0f, 1.0f ) );

	// ����Đ��̐؂�ւ��ʒu��\���c����ݒ�
	timeline->AddLine( mf + num_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ) );
}

//
//�p�^�[���ɂ��J���[�o�[�̕��ʖ��O�̐F�ω�
//
Color4f MotionPlaybackApp2::Pattern_Color(int pattern, int num_segment)
{
	Color4f c;

	switch (pattern)
	{
		case 0: //�S��3�F
			if( 1 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.2f, 1.0f );
			else if( num_segment <= 12 )
				c = Color4f( 0.2f, 0.8f, 0.2f, 1.0f );
			else
				c = Color4f( 0.2f, 0.2f, 0.8f, 1.0f );
			break;
		case 1: //�����Ƌ���2�F
			if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( 7 <= num_segment && num_segment <= 10 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( num_segment == 11 || num_segment == 12 )
				c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 2: //�r2�F
			if( 13 <= num_segment && num_segment <= 35 )
				c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
			else if( 36 <= num_segment )
				c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 3: //�r2�F
			if( 1 <= num_segment && num_segment <= 3 )
				c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
			else if( 4 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 4: //�S��6�F
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
		case 5: //1����1�F
			if( num_segment == view_segment || view_segments[num_segment] == 1 )
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
//�J���[�o�[�̌덷�ɂ��F�̕ω���ݒ�(DTWframe)
//
void MotionPlaybackApp2::ColorBarElementErrorCenterOfGravity(Timeline* timeline, int track_num, vector<vector<float>> error, vector<vector<int>> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio = 0.0f;
	float max_error = 0.0f;
	float min_error = 100.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	Color4f c;

	//frame���ł̍ő�덷�l�����߂�
	for(int f = 0; f < warpingPath[0].size(); f++)
	{
		if(max_error < error[warpingPath[0][f]][warpingPath[1][f]])
		{
			max_error = error[warpingPath[0][f]][warpingPath[1][f]];
			max_frame = f;
		}
		else if (min_error > error[warpingPath[0][f]][warpingPath[1][f]])
			min_error = error[warpingPath[0][f]][warpingPath[1][f]];
	}

	max_error = max_error - min_error;

	//�덷�̑傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f < warpingPath[0].size(); f++)
	{
		red_ratio = (error[warpingPath[0][f]][warpingPath[1][f]] - min_error) / max_error;
		if (red_ratio == 1.0)
			c = Color4f( 0.0f, 0.0f, 0.0f, 1.0f );
		else if(red_ratio > 0.75)
			c = Color4f( 1.0f, (1.0f - red_ratio) * 4.0f, 0.0f, 1.0f );
		else if (red_ratio > 0.5)
			c = Color4f((red_ratio - 0.5f) * 4.0f, 1.0f, 0.0f, 1.0f);
		else if (red_ratio > 0.25)
			c = Color4f(0.0f, 1.0f, (0.5f - red_ratio) * 4.0f, 1.0f);
		else
			c = Color4f( 0.0f, red_ratio * 4.0f, 1.0f, 1.0f );

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementCenterOfGravity_X(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_x = center[warpingPath[0]].x;
	float min_x = center[warpingPath[0]].x;
	float center_x = center[warpingPath[0]].x;
	float name_space = 30.0f;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f < warpingPath.size(); f++)
		if(max_x < center[warpingPath[f]].x)
			max_x = center[warpingPath[f]].x;
		else if (min_x > center[warpingPath[f]].x)
			min_x = center[warpingPath[f]].x;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f < warpingPath.size(); f++)
	{
		if(center[warpingPath[f]].x >= center_x)
		{
			red_ratio = (center[warpingPath[f]].x - center_x) / (max_x - center_x);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].x - min_x) / (center_x - min_x);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementCenterOfGravity_Y(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_y = center[warpingPath[0]].y;
	float min_y = center[warpingPath[0]].y;
	float center_y = center[warpingPath[0]].y;
	float name_space = 30.0f;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f < warpingPath.size(); f++)
		if(max_y < center[warpingPath[f]].y)
			max_y = center[warpingPath[f]].y;
		else if (min_y > center[warpingPath[f]].y)
			min_y = center[warpingPath[f]].y;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f < warpingPath.size(); f++)
	{
		if(center[warpingPath[f]].y >= center_y)
		{
			red_ratio = (center[warpingPath[f]].y - center_y) / (max_y - center_y);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].y - min_y) / (center_y - min_y);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementCenterOfGravity_Z(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_z = center[warpingPath[0]].z;
	float min_z = center[warpingPath[0]].z;
	float center_z = center[warpingPath[0]].z;
	float name_space = 30.0f;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f < warpingPath.size(); f++)
		if(max_z < center[warpingPath[f]].z)
			max_z = center[warpingPath[f]].z;
		else if (min_z > center[warpingPath[f]].z)
			min_z = center[warpingPath[f]].z;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f < warpingPath.size(); f++)
	{
		if(center[warpingPath[f]].z >= center_z)
		{
			red_ratio = (center[warpingPath[f]].z - center_z) / (max_z - center_z);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].z - min_z) / (center_z - min_z);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

//
//�J���[�o�[�̌덷�ɂ��F�̕ω���ݒ�(�Đ��؂�ւ��p)
//
void MotionPlaybackApp2::ColorBarElementRepErrorCenterOfGravity(Timeline* timeline, int track_num, vector<vector<float>> error, vector<vector<int>> warpingPath, Motion& motion, Motion& motion2, float curr_frame)
{
	float red_ratio = 0.0f;
	float max_error = 0.0f;
	float min_error = 100.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	Color4f c;

	//frame���ł̍ő�덷�l�����߂�
	for(int f = 0; f <= mf; f++)
	{
		if(max_error < error[warpingPath[0][f]][warpingPath[1][f]])
		{
			max_error = error[warpingPath[0][f]][warpingPath[1][f]];
			max_frame = f;
		}
		else if (min_error > error[warpingPath[0][f]][warpingPath[1][f]])
			min_error = error[warpingPath[0][f]][warpingPath[1][f]];
	}

	for (int f = mf + 1; f < frames; f++)
	{
		if (max_error < error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)])
		{
			max_error = error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)];
			max_frame = f;
		}
		else if (min_error > error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)])
			min_error = error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)];
	}

	max_error = max_error - min_error;

	//�덷�̑傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f < frames; f++)
	{
		if (f <= mf)
			red_ratio = (error[warpingPath[0][f]][warpingPath[1][f]] - min_error) / max_error;
		else
			red_ratio = (error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)] - min_error) / max_error;
		
		if (red_ratio == 1.0)
			c = Color4f( 0.0f, 0.0f, 0.0f, 1.0f );
		else if(red_ratio > 0.75)
			c = Color4f( 1.0f, (1.0f - red_ratio) * 4.0f, 0.0f, 1.0f );
		else if (red_ratio > 0.5)
			c = Color4f((red_ratio - 0.5f) * 4.0f, 1.0f, 0.0f, 1.0f);
		else if (red_ratio > 0.25)
			c = Color4f(0.0f, 1.0f, (0.5f - red_ratio) * 4.0f, 1.0f);
		else
			c = Color4f( 0.0f, red_ratio * 4.0f, 1.0f, 1.0f );

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementRepCenterOfGravity_X(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_x = center[warpingPath[0]].x;
	float min_x = center[warpingPath[0]].x;
	float center_x = center[warpingPath[0]].x;
	float name_space = 30.0f;
	int kirikae;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f <= mf; f++)
		if(max_x < center[warpingPath[f]].x)
			max_x = center[warpingPath[f]].x;
		else if (min_x > center[warpingPath[f]].x)
			min_x = center[warpingPath[f]].x;

	if(warpingPath[mf] == m1f)
		kirikae = m1f;
	else if(warpingPath[mf] == m2f)
		kirikae = m2f;

	for(int f = mf + 1; f <= frames; f++)
		if(max_x < center[min(motion.num_frames - 1, f - mf + kirikae)].x)
			max_x = center[min(motion.num_frames - 1, f - mf + kirikae)].x;
		else if (min_x > center[min(motion.num_frames - 1, f - mf + kirikae)].x)
			min_x = center[min(motion.num_frames - 1, f - mf + kirikae)].x;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f <= mf; f++)
	{
		if(center[warpingPath[f]].x >= center_x)
		{
			red_ratio = (center[warpingPath[f]].x - center_x) / (max_x - center_x);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].x - min_x) / (center_x - min_x);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
	for(int f = mf + 1; f < frames; f++)
	{
		if(center[min(motion.num_frames - 1, f - mf + kirikae)].x >= center_x)
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].x - center_x) / (max_x - center_x);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].x - min_x) / (center_x - min_x);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementRepCenterOfGravity_Y(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_y = center[warpingPath[0]].y;
	float min_y = center[warpingPath[0]].y;
	float center_y = center[warpingPath[0]].y;
	float name_space = 30.0f;
	int kirikae;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f <= mf; f++)
		if(max_y < center[warpingPath[f]].y)
			max_y = center[warpingPath[f]].y;
		else if (min_y > center[warpingPath[f]].y)
			min_y = center[warpingPath[f]].y;

	if(warpingPath[mf] == m1f)
		kirikae = m1f;
	else if(warpingPath[mf] == m2f)
		kirikae = m2f;

	for(int f = mf + 1; f <= frames; f++)
		if(max_y < center[min(motion.num_frames - 1, f - mf + kirikae)].y)
			max_y = center[min(motion.num_frames - 1, f - mf + kirikae)].y;
		else if (min_y > center[min(motion.num_frames - 1, f - mf + kirikae)].y)
			min_y = center[min(motion.num_frames - 1, f - mf + kirikae)].y;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f <= mf; f++)
	{
		if(center[warpingPath[f]].y >= center_y)
		{
			red_ratio = (center[warpingPath[f]].y - center_y) / (max_y - center_y);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].y - min_y) / (center_y - min_y);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
	for(int f = mf + 1; f < frames; f++)
	{
		if(center[min(motion.num_frames - 1, f - mf + kirikae)].y >= center_y)
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].y - center_y) / (max_y - center_y);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].y - min_y) / (center_y - min_y);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

void MotionPlaybackApp2::ColorBarElementRepCenterOfGravity_Z(Timeline* timeline, int track_num, vector<Point3f> center, vector<int> warpingPath, Motion& motion, float curr_frame)
{
	float red_ratio;
	float max_z = center[warpingPath[0]].z;
	float min_z = center[warpingPath[0]].z;
	float center_z = center[warpingPath[0]].z;
	float name_space = 30.0f;
	int kirikae;
	Color4f c;

	//frame���ł̍ő�ŏ��l�����߂�
	for(int f = 0; f <= mf; f++)
		if(max_z < center[warpingPath[f]].z)
			max_z = center[warpingPath[f]].z;
		else if (min_z > center[warpingPath[f]].z)
			min_z = center[warpingPath[f]].z;

	if(warpingPath[mf] == m1f)
		kirikae = m1f;
	else if(warpingPath[mf] == m2f)
		kirikae = m2f;

	for(int f = mf + 1; f <= frames; f++)
		if(max_z < center[min(motion.num_frames - 1, f - mf + kirikae)].z)
			max_z = center[min(motion.num_frames - 1, f - mf + kirikae)].z;
		else if (min_z > center[min(motion.num_frames - 1, f - mf + kirikae)].z)
			min_z = center[min(motion.num_frames - 1, f - mf + kirikae)].z;

	//�傫���ƐF�t�����ăt���[�����ɕ\��
	for(int f = 0; f <= mf; f++)
	{
		if(center[warpingPath[f]].z >= center_z)
		{
			red_ratio = (center[warpingPath[f]].z - center_z) / (max_z - center_z);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[warpingPath[f]].z - min_z) / (center_z - min_z);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
	for(int f = mf + 1; f < frames; f++)
	{
		if(center[min(motion.num_frames - 1, f - mf + kirikae)].z >= center_z)
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].z - center_z) / (max_z - center_z);

			if(red_ratio >= 0.5)
				c = Color4f( 1.0f, (1.0f - red_ratio) * 2.0f, 0.0f, 1.0f );
			else
				c = Color4f( red_ratio * 2.0f, 1.0f, 0.0f, 1.0f );
		}
		else
		{
			red_ratio = (center[min(motion.num_frames - 1, f - mf + kirikae)].z - min_z) / (center_z - min_z);

			if(red_ratio >= 0.5)
				c = Color4f( 0.0f, 1.0f, (1.0f - red_ratio) * 2.0f, 1.0f );
			else
				c = Color4f( 0.0f, red_ratio * 2.0f, 1.0f, 1.0f );
		}

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", track_num );
	}
}

//
//�J���[�o�[��S�ĊD�F�ɐݒ�(DTWframe)
//
void MotionPlaybackApp2::ColorBarElementGray(Timeline* timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion & motion)
{
	float name_space = 30.0f;
				
	//���O�����\��
	for(int i = 0; i < PassAll[0].size(); i++)
		if(i == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
}

//
//�J���[�o�[��S�ĊD�F�ɐݒ�(�Đ��؂�ւ��p)
//
void MotionPlaybackApp2::ColorBarElementRepGray(Timeline* timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion& motion)
{
	float name_space = 30.0f;
				
	//���O�����\��
	for(int f = 0; f < frames; f++)
		if(f == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
}

//
//DTW������
//
void DTWinformation2::DTWinformation_init( int frames1, int frames2, const Motion & motion1, const Motion & motion2 )
{
	//i���̐߁Aj���t���[��1�Ak���t���[��2
	int numSegments = motion1.body->num_segments;

	centerOfGravity1.resize(frames1, {0.0f, 0.0f, 0.0f});
	centerOfGravity2.resize(frames2, {0.0f, 0.0f, 0.0f});
	errorCenterOfGravity.resize(frames1,vector<float>(frames2, 0.0f));

	vector< Matrix4f >  seg_frame_array1, seg_frame_array2;
	Matrix4f  mat1, mat2;
	vector< vector< Vector3f > > v1(frames1, vector< Vector3f >(numSegments));
	vector< vector< Vector3f > > v2(frames2, vector< Vector3f >(numSegments));

	//���^���w�v�Z
	for(int j = 0; j < frames1; j++)
	{
		ForwardKinematics( motion1.frames[ j ], seg_frame_array1);
		for(int k = 0; k < frames2; k++)
		{
			ForwardKinematics( motion2.frames[ k ], seg_frame_array2);
			for(int i = 0; i < numSegments; i++)
			{
				//��̕��ʂ͌v�Z���Ȃ�
				while(i > 16 && i < 36)
					i++;
				if(i > 39)
					break;

				// �̐߂̒��S�̈ʒu�E��������Ƃ���ϊ��s���K�p
				mat1.set( seg_frame_array1[ i ] );
				mat1.get(&v1[j][i]);

				mat2.set( seg_frame_array2[ i ] );
				mat2.get(&v2[k][i]);
			}
		}
	}

	//�g�̏d�S�덷�̌v�Z
	for(int j = 0; j < frames1; j++)
		for(int k = 0; k < frames2; k++)
			errorCenterOfGravity[j][k] = ErrorCulculateCenterOfGravity(v1[j], v2[k], &centerOfGravity1[j], &centerOfGravity2[k]);

	for(int j = 0; j < frames1; j++)
		std::cout << centerOfGravity1[j] << std::endl;

	//�g�̏d�S�덷�̑S�̂ɑ΂���p�X�̍쐬
	int f1 = frames1 - 1, f2 = frames2 - 1;
	warpingPath.resize(2, vector<int>());
	warpingPath[0].push_back(f1);
	warpingPath[1].push_back(f2);
	float error1, error2, error3;

	while (f1 >= 0 && f2 >= 0)
	{
		if (f1 == 0 && f2 == 0)
			break;
		else if(f1 == 0 && f2 != 0)
		{
			warpingPath[0].push_back(f1);
			warpingPath[1].push_back(--f2);
		}
		else if (f1 != 0 && f2 == 0)
		{
			warpingPath[0].push_back(--f1);
			warpingPath[1].push_back(f2);
		}
		else
		{
			error1 = errorCenterOfGravity[f1 - 1][f2 - 1] + Cost(f1 - 1, f2 - 1);
			error2 = errorCenterOfGravity[f1 - 1][f2] + Cost(f1 - 1, f2);
			error3 = errorCenterOfGravity[f1][f2 - 1] + Cost(f1, f2 - 1);
			if (error1 > error2 || error1 > error3)
				if (error2 > error3)
				{
					warpingPath[0].push_back(f1);
					warpingPath[1].push_back(--f2);
				}
				else
				{
					warpingPath[0].push_back(--f1);
					warpingPath[1].push_back(f2);
				}
			else
			{
				warpingPath[0].push_back(--f1);
				warpingPath[1].push_back(--f2);
			}
		}
	}

	std::reverse(warpingPath[0].begin(), warpingPath[0].end());
	std::reverse(warpingPath[1].begin(), warpingPath[1].end());
	errorFrame = warpingPath[0].size();

	//�g�̏d�S�덷�̃p�X�̕\��
	std::cout << "ErrorCenterOfGravity"<< std::endl;
	for(int f = 0; f < errorFrame; f++)
		std::cout <<  f << "[" << warpingPath[0][f] << "," <<
		warpingPath[1][f] << "]" << errorCenterOfGravity[warpingPath[0][f]][warpingPath[1][f]] << std::endl;
	std::cout << "motion1.num_frames:" << frames1 << std::endl <<
		"motion2.num_frames:" << frames2 << std::endl;
}

float  DTWinformation2::ErrorCulculateCenterOfGravity(vector< Vector3f > v1, vector< Vector3f > v2, Point3f * centerOfGravity1, Point3f * centerOfGravity2)
{
	Vector3f head1, chest1, rightUpperArm1, rightCalf1, leftUpperArm1, leftCalf1, rightThigh1, leftThigh1, rightForeArm1, leftForeArm1,
		head2, chest2, rightUpperArm2, rightCalf2, leftUpperArm2, leftCalf2, rightThigh2, leftThigh2, rightForeArm2, leftForeArm2;
	float error;

	head1.interpolate(v1[11],v1[12],0.5f);
	head1.scale(0.08f);
		(*centerOfGravity1).set(head1);
	chest1.interpolate(v1[10],v1[9],0.5f);
	chest1.interpolate(chest1,v1[8],0.5f);
	chest1.interpolate(chest1,v1[0],0.5f);
	chest1.scale(0.497f);
		(*centerOfGravity1).add(chest1);
	rightUpperArm1.interpolate(v1[13],v2[14],0.5f);
	rightUpperArm1.scale(0.028f);
		(*centerOfGravity1).add(rightUpperArm1);
	rightCalf1.interpolate(v1[2],v1[3],0.5f);
	rightCalf1.scale(0.061f);
		(*centerOfGravity1).add(rightCalf1);
	leftUpperArm1.interpolate(v1[36],v1[37],0.5f);
	leftUpperArm1.scale(0.028f);
		(*centerOfGravity1).add(leftUpperArm1);
	leftCalf1.interpolate(v1[5],v1[6],0.5f);
	leftCalf1.scale(0.061f);
		(*centerOfGravity1).add(leftCalf1);
	rightThigh1.scale(0.10f,v1[1]);
		(*centerOfGravity1).add(rightThigh1);
	rightForeArm1.interpolate(v1[15],v1[16],0.5f);
	rightForeArm1.scale(0.022f);
		(*centerOfGravity1).add(rightForeArm1);
	leftForeArm1.interpolate(v1[38],v1[39],0.5f);
	leftForeArm1.scale(0.022f);
		(*centerOfGravity1).add(leftForeArm1);
	leftThigh1.scale(0.10f,v1[4]);
		(*centerOfGravity1).add(leftThigh1);

	head2.interpolate(v2[11],v2[12],0.5f);
	head2.scale(0.08f);
		(*centerOfGravity2).set(head2);
	chest2.interpolate(v2[10],v2[9],0.5f);
	chest2.interpolate(chest2,v2[8],0.5f);
	chest2.interpolate(chest2,v2[0],0.5f);
	chest2.scale(0.497f);
		(*centerOfGravity2).add(chest2);
	rightUpperArm2.interpolate(v2[13],v2[14],0.5f);
	rightUpperArm2.scale(0.028f);
		(*centerOfGravity2).add(rightUpperArm2);
	rightCalf2.interpolate(v2[2],v2[3],0.5f);
	rightCalf2.scale(0.061f);
		(*centerOfGravity2).add(rightCalf2);
	leftUpperArm2.interpolate(v2[36],v2[37],0.5f);
	leftUpperArm2.scale(0.028f);
		(*centerOfGravity2).add(leftUpperArm2);
	leftCalf2.interpolate(v2[5],v2[6],0.5f);
	leftCalf2.scale(0.061f);
		(*centerOfGravity2).add(leftCalf2);
	rightThigh2.scale(0.10f,v2[1]);
		(*centerOfGravity2).add(rightThigh2);
	rightForeArm2.interpolate(v2[15],v2[16],0.5f);
	rightForeArm2.scale(0.022f);
		(*centerOfGravity2).add(rightForeArm2);
	leftForeArm2.interpolate(v2[38],v2[39],0.5f);
	leftForeArm2.scale(0.022f);
		(*centerOfGravity2).add(leftForeArm2);
	leftThigh2.scale(0.10f,v2[4]);
		(*centerOfGravity2).add(leftThigh2);

	error = (*centerOfGravity1).distance(*centerOfGravity2);

	return error;
}

float DTWinformation2::Cost(int frame1, int frame2)
{
	return fabs(frame1 - frame2) * 0.002f;
}
