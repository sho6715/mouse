//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���}�~
#ifndef _HAL_DIST_H
#define _HAL_DIST_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O

//**************************************************
// ��`�idefine�j
//**************************************************
/* �����Z���T */
#define		LED_DIST_RF		( PORTA.PODR.BIT.B6 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_RS		( PORTA.PODR.BIT.B4 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_LF		( PORTA.PODR.BIT.B1 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_LS		( PORTA.PODR.BIT.B3 )			// �r�b�g�A�N�Z�X

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* �����Z���TID  */
typedef enum{
	DIST_SEN_R_FRONT = 0,			// �E�O
	DIST_SEN_L_FRONT,			// ���O
	DIST_SEN_R_SIDE,			// �E��
	DIST_SEN_L_SIDE,			// ����
	DIST_SEN_NUM
}enDIST_SEN_ID;

/*�����Z���T�|�[�����O�^�C�v*/
typedef enum{
	DIST_POL_FRONT = 0,	//�O��
	DIST_POL_SIDE,		//����
	DISR_POL_MAX
}enDIST_POL;

/*�����Z���T������*/
typedef enum{
	DIST_STANDBAY = 0,	//�X�^���o�C
	DIST_NO_CTRL,		//����s�v
	DIST_NO_WALL,		//�ǂȂ�
	DIST_FRONT,		//�O�ǐ���
	DIST_RIGHT,		//�E�ǐ���
	DIST_LEFT,		//���ǐ���
	DIST_STATE_MAX
}enDIST_SEN_STATE;

//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************




//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
/* �����Z���T */
PUBLIC void DIST_Pol_Front( void );
PUBLIC void DIST_Pol_Side( void );
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id );
PUBLIC void DIST_getErr( LONG* p_err );

PUBLIC BOOL DIST_isWall_FRONT( void );
PUBLIC BOOL DIST_isWall_R_SIDE( void );
PUBLIC BOOL DIST_isWall_L_SIDE( void );

PUBLIC void Dist_autocalibration (void);
PUBLIC void calibration(void);

#endif
