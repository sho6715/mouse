// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����search�w�b�_�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.08.25			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���}�~
#ifndef _SEARCH_H
#define _SEARCH_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <hal.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define MazeSize_x		(16)
#define MazeSize_y		(16)
#define Goal_x			(4)
#define Goal_y			(7)


#define UNKNOWN	2
#define NOWALL	0
#define WALL	1
#define VWALL	3

#define MASK_SEARCH	0x01	
#define MASK_SECOND	0x03	

#define CONV_SEN2WALL(w) ((w) ? WALL : NOWALL)

//**************************************************
// �񋓑́ienum�j
//**************************************************
typedef enum
{
	north = 0,
	east = 1,
	south = 2,
	west = 3,
}en_direction;

typedef enum
{
	front = 0,
	right = 1,
	rear = 2,
	left = 3,
	unknown,
}en_local_dir;

//**************************************************
// �\���́istruct�j
//**************************************************
typedef struct
{
	UCHAR	north:2;
	UCHAR	east:2;
	UCHAR	south:2;
	UCHAR	west:2;
}t_wall;

typedef struct
{
	short x;
	short y;
	en_direction dir;
}t_position;

//**************************************************
// �O���[�o���ϐ�
//**************************************************
extern PUBLIC UCHAR	map[MazeSize_x][MazeSize_y];
extern PUBLIC t_wall	wall[MazeSize_x][MazeSize_y];
PRIVATE FLOAT 		f_MotSearchMaxSpeed 		= 300.0f;		// �T�����ڕW���x
extern PUBLIC t_position	mypos;


//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void init_maze(void);
PUBLIC void clear_map(int x, int y);
PUBLIC void make_map(int x, int y,int mask);
PUBLIC void set_wall(int x, int y);
PUBLIC UCHAR is_unknown(int x, int y);
PUBLIC int get_priority(int x, int y, en_direction dir );
PUBLIC int get_nextdir(int x, int y, int mask, en_direction *dir);
PUBLIC void search_lefthand(void);
PUBLIC void search_adachi(int gx, int gy);
PUBLIC void goal_appeal(void);
PUBLIC void fast_run(int x, int y,float f_speed);
PUBLIC void search_adachi_hitwall(int gx,int gy);

PUBLIC void search_adachi_sura(int gx,int gy);

#endif //_SEARCH_H