// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <hal.h>							// HAL
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <parameters.h>
#include <hal_dist.h>
//#include <search.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define		BAT_GOOD			(2135)			// �c�ʌ����Ă����i���F�j�A1�Z��3.7V�ȏ�F 2135 = ( 3700mV * 2�Z�� ) / 4.3(����) / 3300 * 4096 - 1
#define		BAT_LOW				(1961)			// �c�ʂ�΂��I�I�i�ԐF�j�A1�Z��3.4V�ȏ�F 1961 = ( 3400mV * 2�Z�� ) / 4.3(����) / 3300 * 4096 - 1
#define		GYRO_REF_NUM		(200)		//�W���C���̃��t�@�����X�l���T���v�����O���鐔
#define		ACCEL_REF_NUM		(50)		//�����x�̃��t�@�����X�l���T���v�����O���鐔
#define		ENC_RESET_VAL		(32768)			// �G���R�[�_�̒��Ԓl
#define		ENC_R_TSTR			(TPUA.TSTR.BIT.CST1)	// �E�G���R�[�_�p���X�J�E���g�J�n
#define		ENC_L_TSTR			(TPUA.TSTR.BIT.CST2)	// ���G���R�[�_�p���X�J�E���g�J�n
#define		DCM_ENA				(PORTB.PODR.BIT.B7)		// DCM�̗L��/����
#define		DCM_R_IN1			(PORTB.PODR.BIT.B1)		// DCM�EIN1
#define		DCM_R_IN2			(PORTB.PODR.BIT.B3)		// DCM�EIN2
#define		DCM_L_IN1			(PORTB.PODR.BIT.B5)		// DCM��IN1
#define		DCM_L_IN2			(PORTB.PODR.BIT.B6)		// DCM��IN2
#define		DCM_R_TIMER			(TPUA.TSTR.BIT.CST0)	// DCM�E�^�C�}�J�n
#define		DCM_L_TIMER			(TPUA.TSTR.BIT.CST3)	// DCM���^�C�}�J�n
#define		DCM_R_TIORA			(TPU0.TIORH.BIT.IOA)	// DCM�E�s���o�͐ݒ�A
#define		DCM_R_TIORB			(TPU0.TIORH.BIT.IOB)	// DCM�E�s���o�͐ݒ�B
#define		DCM_L_TIORA			(TPU3.TIORH.BIT.IOA)	// DCM���s���o�͐ݒ�A
#define		DCM_L_TIORB			(TPU3.TIORH.BIT.IOB)	// DCM���s���o�͐ݒ�B
#define		DCM_R_TCNT			(TPU0.TCNT)				// DCM�E�J�E���g�l
#define		DCM_L_TCNT			(TPU3.TCNT)				// DCM���J�E���g�l
#define		DCM_R_GRA			(TPU0.TGRA)				// DCM�E����
#define		DCM_R_GRB			(TPU0.TGRB)				// DCM�EDuty��
#define		DCM_L_GRA			(TPU3.TGRA)				// DCM������
#define		DCM_L_GRB			(TPU3.TGRB)				// DCM��Duty��

/* �񒲐��p�����[�^ */
#define PI							( 3.14159f )								// ��

/* �����p�����[�^ */
#define VCC_MAX						( 8.4f )									// �o�b�e���ő�d��[V]�A4.2[V]�~2[�Z��]
#define TIRE_R						( 22.1f )									// �^�C�����a [mm]
#define GEAR_RATIO					( 36 / 8 )									// �M�A��(�X�p�[/�s�j�I��)
#define ROTATE_PULSE					( 2048 )									// 1���̃^�C���p���X��
#define DIST_1STEP					( PI * TIRE_R / GEAR_RATIO / ROTATE_PULSE )				// 1�p���X�Ői�ދ��� [mm]
#define F_CNT2MM(cnt)					( (FLOAT)cnt * DIST_1STEP )				// [�J�E���g�l]����[mm]�֊��Z
#define MOT_MOVE_ST_THRESHOLD				( 25 )							// ���i�ړ�������臒l[mm]
#define MOT_MOVE_ST_MIN					( 20 )							// ���i�ړ������̍Œ�ړ���[mm]
//#define MOT_ACC						( 1800 )						// ���i�ړ��̉����x[mm/s2]
//#define MOT_DEC						( 1800 )						// ���i�ړ��̌����x[mm/s2]
//#define MOT_ACC_ANGLE					( 1800 )		//����̊p�����x[mm/s2]
//#define MOT_DEC_ANGLE					( 1800 )		//����̊p�����x[mm/s2]

//20170815 ���M�n����������ɒǉ�
#define A1_MIN					( 25 )						// ��1�Œ�ړ��p�x
#define A2_MIN					( 30 )						// ��2�Œ�ړ��p�x
#define A3_MIN					( 20 )						// ��3�Œ�ړ��p�x

#define ANGLE_OFFSET1_R				( 0 )	//-12					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET1				( 0 )	//-12					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET2_R				( 0 )	//3
#define ANGLE_OFFSET2				( 0 )						// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET3				( 0 )					// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j

//#define MOT_MOVE_ST_THRESHOLD			( 25 )						// ���i�ړ�������臒l[mm]
//#define MOT_MOVE_ST_MIN				( 20 )						// ���i�ړ������̍Œ�ړ���[mm]


#define log_num			(1000)					//���O�擾���i�ύX���͂������ύX�j

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ���䓮��^�C�v */
typedef enum{
	CTRL_ACC,				// [00] ������(���i)
	CTRL_CONST,				// [01] ������(���i)
	CTRL_DEC,				// [02] ������(���i)

	CTRL_SKEW_ACC,			// [03] �΂߉�����(���i)
	CTRL_SKEW_CONST,		// [04] �΂ߓ�����(���i)
	CTRL_SKEW_DEC,			// [05] �΂ߌ�����(���i)
	
	CTRL_HIT_WALL,			// [06]�Ǔ��ē���
	
	CTRL_ACC_TRUN,			// [07] ������(���M�n����)
	CTRL_CONST_TRUN,		// [08] ������(���M�n����)
	CTRL_DEC_TRUN,			// [09] ������(���M�n����)
	
	CTRL_ENTRY_SURA,		// [10]�X�����[���O�O�i
	CTRL_ACC_SURA,			// [11] ������(�X��)
	CTRL_CONST_SURA,		// [12] ������(�X��)
	CTRL_DEC_SURA,			// [13] ������(�X��)
	CTRL_EXIT_SURA,			// [14] �X�����[����O�i

	CTRL_MAX,

}enCTRL_TYPE;


/* ����^�C�v */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,			// [01] ��`����
	MOT_ACC_CONST_DEC_CUSTOM,	// [02] ��`�����i�����l�ύX�j
	MOT_ACC_CONST,				// [03] �����{����
	MOT_ACC_CONST_CUSTOM,		// [04] �����{�����i�����l�ύX�j
	MOT_CONST_DEC,				// [05] �����{����
	MOT_CONST_DEC_CUSTOM,		// [06] �����{�����i�����l�ύX�j
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* ���i�^�C�v */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// �ʏ�̒��i
	MOT_GO_ST_SKEW,				// �΂߂̒��i
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// �\���́istruct�j
//**************************************************
/* ������ */
typedef struct{

	FLOAT			f_time;			// ����					[msec]

	/* ���x���� */
	FLOAT			f_acc1;			// �����x1				[mm/s2]
	FLOAT			f_acc3;			// �����x3				[mm/s2]
	FLOAT			f_now;			// ���ݑ��x				[mm/s]
	FLOAT			f_trgt;			// ������̖ڕW���x		[mm/s]
	FLOAT			f_last;			// ������̍ŏI���x		[mm/s]

	/* �������� */
	FLOAT			f_dist;			// �ړ�����				[mm]
	FLOAT			f_l1;			// ��1�ړ�����			[mm]
	FLOAT			f_l1_2;			// ��1+2�ړ�����		[mm]

	/* �p���x���� */
	FLOAT			f_accAngleS1;	// �p�����x1			[rad/s2]
	FLOAT			f_accAngleS3;	// �p�����x3			[rad/s2]
	FLOAT			f_nowAngleS;	// ���݊p���x			[rad/s]
	FLOAT			f_trgtAngleS;	// ������̖ڕW�p���x	[rad/s]
	FLOAT			f_lastAngleS;	// ������̍ŏI�p���x	[rad/s]

	/* �p�x���� */
	FLOAT			f_angle;		// �ړ��p�x				[rad]
	FLOAT			f_angle1;		// ��1�ړ��p�x			[rad]
	FLOAT			f_angle1_2;		// ��1+2�ړ��p�x		[rad]
}stMOT_DATA;

/* ����f�[�^ */
typedef struct{
	enCTRL_TYPE		en_type;		// ����^�C�v
	FLOAT			f_time;			// �ڕW���� [sec]
	FLOAT			f_acc;			// [���x����]   �����x[mm/s2]
	FLOAT			f_now;			// [���x����]   ���ݑ��x[mm/s]
	FLOAT			f_trgt;			// [���x����]   �ŏI���x[mm/s]
	FLOAT			f_nowDist;		// [��������]   ���݋���[mm]
	FLOAT			f_dist;			// [��������]   �ŏI����[mm]
	FLOAT			f_accAngleS;	// [�p���x����] �p�����x[rad/s2]
	FLOAT			f_nowAngleS;	// [�p���x����] ���݊p���x[rad/s]
	FLOAT			f_trgtAngleS;	// [�p���x����] �ŏI�p���x[rad/s]
	FLOAT			f_nowAngle;		// [�p�x����]   ���݊p�x[rad]
	FLOAT			f_angle;		// [�p������]   �ŏI�p�x[rad]
}stCTRL_DATA;


//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �o�b�e���Ď� */
PRIVATE USHORT	us_BatLvAve = 4095;							// �o�b�e�����ϒl�iAD�ϊ��̍ő�l�ŏ������j

/*�W���C���Z���T*/
PRIVATE SHORT s_GyroVal; 					  				// �W���C���Z���T�̌��ݒl
//PRIVATE SHORT s_GyroValBuf[8];								// �W���C���Z���T�̃o�b�t�@�l
PUBLIC FLOAT  f_GyroNowAngle;		 						// �W���C���Z���T�̌��݊p�x
PRIVATE LONG  l_GyroRef; 									// �W���C���Z���T�̊�l

/*�p���x�擾*/
PRIVATE SHORT s_AccelVal; 					  				// �����x�̎擾�l
PRIVATE FLOAT f_NowAccel;										// �����x�̌��ݒn
PRIVATE LONG  l_AccelRef; 									// �����x�̊�l

/* ����  */
PRIVATE enCTRL_TYPE		en_Type;						// �������
PRIVATE UCHAR 			uc_CtrlFlag			= FALSE;	// �t�B�[�h�o�b�N or �t�B�[�h�t�H���[�h ����L���t���O�iFALSE:�����A1�F�L���j
PRIVATE LONG			l_CntR;							// �E���[�^�̃J�E���g�ω���						�i1[msec]���ɍX�V�����j
PRIVATE LONG			l_CntL;							// �����[�^�̃J�E���g�ω���						�i1[msec]���ɍX�V�����j
// ����
PUBLIC  FLOAT			f_Time 				= 0;		// ���쎞��[sec]								�i1[msec]���ɍX�V�����j
PUBLIC  FLOAT			f_TrgtTime 			= 1000;		// ����ڕW���� [msec]							�i�ݒ�l�j
// ���x����//////////////////////////////////////////
PRIVATE FLOAT 			f_Acc			= 0;		// [���x����]   �����x							�i�ݒ�l�j
PRIVATE FLOAT			f_BaseSpeed		= 0;		// [���x����]   �����x							�i�ݒ�l�j
PRIVATE FLOAT			f_LastSpeed 		= 0;		// [���x����]   �ŏI�ڕW���x					�i�ݒ�l�j
PRIVATE FLOAT			f_NowSpeed		= 0;		// [���x����]   ���݂̑��x [mm/s]				�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_TrgtSpeed 		= 0;		// [���x����]   �ڕW�ړ����x [mm/s]				�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrSpeedBuf		= 0;		// [���x����] �@���x�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_SpeedErrSum 		= 0;		// [���x����]   ���x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// ��������
PRIVATE FLOAT			f_BaseDist		= 0;		// [��������]   �����ʒu						�i�ݒ�l�j
PRIVATE FLOAT			f_LastDist 		= 0;		// [��������]   �ŏI�ړ�����					�i�ݒ�l�j
PUBLIC FLOAT			f_TrgtDist 		= 0;		// [��������]   �ڕW�ړ�����					�i1[msec]���ɍX�V�����j
PUBLIC volatile FLOAT 		f_NowDist		= 0;		// [��������]   ���݋���						�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_NowDistR		= 0;		// [��������]   ���݋����i�E�j					�i1[msec]���ɍX�V�����j
PRIVATE FLOAT 			f_NowDistL		= 0;		// [��������]   ���݋����i���j					�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_DistErrSum 		= 0;		// [��������]   �����ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �p���x����
PRIVATE FLOAT 			f_AccAngleS		= 0;		// [�p���x����] �p�����x						�i�ݒ�l�j
PRIVATE FLOAT			f_BaseAngleS		= 0;		// [�p���x����] �����p���x						�i�ݒ�l�j
PRIVATE FLOAT			f_LastAngleS 		= 0;		// [�p���x����] �ŏI�ڕW�p���x					�i�ݒ�l�j
PUBLIC FLOAT			f_TrgtAngleS 		= 0;		// [�p���x����] �ڕW�p���x [rad/s]				�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrAngleSBuf		= 0;		// [�p���x����] �p���x�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_AngleSErrSum 		= 0;		// [�p���x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �p�x����
PRIVATE FLOAT			f_BaseAngle		= 0;		// [�p�x����]   �����p�x						�i�ݒ�l�j
PRIVATE FLOAT			f_LastAngle 		= 0;		// [�p�x����]   �ŏI�ڕW�p�x					�i�ݒ�l�j
PUBLIC volatile FLOAT 		f_NowAngle		= 0;		// [�p�x����]   ���݊p�x�@	volatile�����Ȃ���while���甲�����Ȃ��Ȃ�i�œK���̂����j�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_TrgtAngle 		= 0;		// [�p�x����]   �ڕW�p�x						�i1[msec]���ɍX�V�����j
PUBLIC FLOAT			f_AngleErrSum 		= 0;		// [�p�x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
// �ǐ���
PRIVATE LONG 			l_WallErr 		= 0;		// [�ǐ���]     �ǂƂ̕΍�						�i1[msec]���ɍX�V�����j
PRIVATE FLOAT			f_ErrDistBuf		= 0;		// [�ǐ���]     �����Z���T�[�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j

/* ���� */
PRIVATE FLOAT 			f_MotNowSpeed 		= 0.0f;		// ���ݑ��x
PRIVATE FLOAT 			f_MotTrgtSpeed 		= 0.0f;		// �ڕW���x
PRIVATE	stMOT_DATA 		st_Info;				// �V�[�P���X�f�[�^
PRIVATE FLOAT			f_MotSuraStaSpeed	= 0.0f;
PRIVATE FLOAT			sliplengs		= 0.0f;		//�X���b�v����

PRIVATE enMOT_WALL_EDGE_TYPE	en_WallEdge = MOT_WALL_EDGE_NONE;	// �ǐ؂�␳
PRIVATE BOOL			bl_IsWallEdge = FALSE;				// �ǐ؂ꌟ�m�iTRUE:���m�AFALSE�F�񌟒m�j
PRIVATE FLOAT			f_WallEdgeAddDist = 0;				// �ǐ؂�␳�̈ړ�����

//�t�F�C���Z�[�t
PUBLIC FLOAT  f_ErrChkAngle; 			  // �W���C���Z���T�̃G���[���o�p�̊p�x
PUBLIC BOOL   bl_ErrChk; 				  // �W���C���Z���T�̃G���[���o�iFALSE�F���m���Ȃ��ATRUE�F���m����j
PRIVATE BOOL			bl_failsafe		= FALSE;	// �}�E�X���̐���s�\�iTRUE�F����s�\�AFALSE�F����\�j

//���O�v���O�����Q�i�擾���ύX��define�ցj
PRIVATE	FLOAT	Log_1[log_num];
PRIVATE FLOAT	Log_2[log_num];
PRIVATE FLOAT	Log_3[log_num];
PRIVATE FLOAT	Log_4[log_num];
PRIVATE FLOAT	Log_5[log_num];
PRIVATE FLOAT	Log_6[log_num];
PRIVATE FLOAT	Log_7[log_num];
PRIVATE FLOAT	Log_8[log_num];
PRIVATE FLOAT	Log_9[log_num];
PRIVATE FLOAT	Log_10[log_num];
PRIVATE FLOAT	Log_11[log_num];
PRIVATE FLOAT	Log_12[log_num];

PRIVATE	USHORT	log_count = 0;
PUBLIC	BOOL	b_logflag = FALSE;

PRIVATE FLOAT	templog1	= 0;
PRIVATE FLOAT	templog2	= 0;

//���O�p�f���[�e�B�[
PRIVATE	FLOAT	f_Duty_R;
PRIVATE	FLOAT	f_Duty_L;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

//���O��p�ϐ�
//PRIVATE FLOAT LogBuf[1000];
// *************************************************************************
//   �@�\		�F ���O���֐�
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.5.10		��	�V�K
// *************************************************************************/
/*PUBLIC void log_in(FLOAT log_input)
{

	int i=0;
	//�o�b�t�@���V�t�g
	while(i < 1000){
		LogBuf[i] = LogBuf[i+1];
		i++;
	}
	LogBuf[999]=log_input;

}
*/
// *************************************************************************
//   �@�\		�F ���O�o�͊֐�
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.5.10		��	�V�K
// *************************************************************************/
/*PUBLIC void log_out(void)
{
	int i=0;
	while(i<1000){
		printf("%5.2f\n\r",LogBuf[i]);
		i++;
	}
}
*/
// *************************************************************************
//   �@�\		�F HAL������������B
//   ����		�F �Ȃ�
//   ����		�F �����ϐ��Ȃǂ��N���A����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.27			�O��			�V�K
// 		v1.1		2013.11.27			�O��			�Z���T�̊�l�ƃ��~�b�g�l��ݒ�
// *************************************************************************/
PUBLIC void HAL_init( void )
{
	/* �W���C���Z���T */
	f_GyroNowAngle = 0;			// �W���C���Z���T�̌��݊p�x(0�ɂ��Ă��T�����͓������A����Ƃ�testrun�Ƃ��͓����Ȃ�)�C���ς݂Ǝv����
	l_GyroRef  = 0;				// �W���C���Z���T�̊�l
	
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;
	
	/* �G���R�[�_ */
	ENC_Sta();	//�����̃s����LEFT(�ˋN�͑O������)
	
	/* DCM*/
	DCM_ENA = ON;
	
	/* [�b��] warning�����������i�폜����OK�j */
	f_AccAngleS = f_AccAngleS;
	f_BaseAngleS = f_BaseAngleS;
	f_LastAngleS = f_LastAngleS;
	f_BaseAngle = f_BaseAngle;
	f_LastAngle = f_LastAngle;

}

// *************************************************************************
//   �@�\		�F 1�����o��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_putch(char data)
{
	while(SCI1.SSR.BIT.TEND == 0) {};
//		if (SCI1.SSR.BYTE & 0x80) {			// ���M�o�b�t�@�̋󂫃`�F�b�N
			SCI1.TDR = data;
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x40;
			SCI1.SSR.BIT.TEND = 0;
//			break;
//		}
//	}
}


// *************************************************************************
//   �@�\		�F ������o��
//   ����		�F �Ȃ�
//   ����		�F "\n"�݂̂�"CR+LF"�o�͂��s���B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_puts(char *buffer)
{
	char data;
	
	/* null�܂ŏo�� */
	while( (data = *( buffer++ ) ) != 0 ){
		
		/* �f�[�^�̒l�ɉ����ďo�͂�ς��� */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR�o��
			SCI_putch(0x0a);		// LF�o��
		} else {
			SCI_putch(data);		// 1�����o��
		}
	}
}


// *************************************************************************
//   �@�\		�F ������o��
//   ����		�F �Ȃ�
//   ����		�F �����񒷂��w��t��"/\n"�̂� "CR+LF"�o�͂��s���B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void SCI_putsL(char *buffer, int len)
{
	int i;
	char data;
	
	for( i=0; i<len; i++ ){
		data=*(buffer++);
		
		/* �f�[�^�̒l�ɉ����ďo�͂�ς��� */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR�o��
			SCI_putch(0x0a);		// LF�o��
		} else {
			SCI_putch(data);		// 1�����o��
		}
	}
}


// *************************************************************************
//   �@�\		�F 1�����o�͗p���b�p�[�֐�
//   ����		�F �Ȃ�
//   ����		�F printf�Ȃǂ̒჌�x���o�͂Ɏg�p�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC void charput(unsigned char data)
{
	SCI_putch(data);
}


// *************************************************************************
//   �@�\		�F ���̓o�b�t�@�`�F�b�N
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC int SCI_chkRecv(void)
{
	/* �f�[�^�̎�M�`�F�b�N */
	if (IR(SCI1,RXI1) == 1) {
		return 1;		// ��M�f�[�^����
	}
	else {
		return 0;		// ��M�f�[�^�Ȃ�
	}
}


// *************************************************************************
//   �@�\		�F 1�������́ichar�^�Łj
//   ����		�F �Ȃ�
//   ����		�F �G�R�[�o�b�N�Ȃ��B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC char SCI_getch(void)
{
	char data;
	
	while(1){
		/* �f�[�^�̎�M�`�F�b�N */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// �f�[�^��M
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   �@�\		�F 1�������́iunsigned char�^�Łj
//   ����		�F �Ȃ�
//   ����		�F �G�R�[�o�b�N�Ȃ��B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC unsigned char SCI_getch_uc(void)
{
	unsigned char data;
	while(1){
		/* �f�[�^�̎�M�`�F�b�N */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// �f�[�^��M
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   �@�\		�F ���������
//   ����		�F �Ȃ�
//   ����		�F CR�R�[�h�܂�/�ő�255����/�G�R�[�o�b�N����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC int SCI_gets(char *buffer)
{
	char data;
	int  i = 0;

	while(1){
		data = SCI_getch();		// 1��������
		*buffer = data;
		SCI_putch(data);		// 1�����o��(�G�R�[�o�b�N)
		buffer++;
		i++;
		if (i > 255)      break;	// �ő啶�����ɓ��B
		if (data == 0x0D) break;	// CR�R�[�h�܂Ŏ�M����
	}
	*buffer = (unsigned char)0;		// null
	
	return i;						// ���͕�������ԋp
}


// *************************************************************************
//   �@�\		�F 1�������͗p���b�p�[�֐�
//   ����		�F �Ȃ�
//   ����		�F scanf�Ȃǂ̒჌�x���o�͂Ɏg�p�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC unsigned char charget(void)
{
	return SCI_getch_uc();
}


// *************************************************************************
//   �@�\		�F �o�b�e���d�����擾����
//   ����		�F �Ȃ�
//   ����		�F ���O5��̕��ϒl
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �d��[mV]
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT BAT_getLv(void)
{
	FLOAT f_val = (FLOAT)( us_BatLvAve + 1 );		// �l��0����n�܂邩��1�����Z
	
	return ( f_val / 4096 * 3300 * 4.4f );
}


// *************************************************************************
//   �@�\		�F �o�b�e���Ď��p�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.16			�O��			�V�K
// *************************************************************************/
PUBLIC void BAT_Pol( void )
{
	static USHORT 	us_batLv[5] = { 4095, 4095, 4095, 4095, 4095 };		// �o�b�e�����x���iAD�ϊ��̍ő�l�ŏ������j

	/* ================================================== */
	/*  ���ϒl���擾���邽�߁A�f�[�^�̃o�b�t�@�������s��  */
	/* ================================================== */
	/* �o�b�t�@���V�t�g */
	us_batLv[4] = us_batLv[3];
	us_batLv[3] = us_batLv[2];
	us_batLv[2] = us_batLv[1];
	us_batLv[1] = us_batLv[0];

	/* �ŐV�̒l���i�[ */
	S12AD.ADANS0.WORD 		= 0x0001;		// AN0 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	us_batLv[0] = S12AD.ADDR0;				// AN0 �ϊ��f�[�^�擾

	/* �d�����ω� */
	us_BatLvAve = ( us_batLv[0] + us_batLv[1] + us_batLv[2] + us_batLv[3] + us_batLv[4] ) / 5;
	
	/*  �c�ʂɉ�����LED��\��  */
	/* ======================= */
	if( us_BatLvAve < BAT_LOW ) {			// �c�ʂ�΂��I�I�i�ԐF�j
		LEDG = OFF;
		LEDR = ON;
	}
	else if( us_BatLvAve < BAT_GOOD ) {		// �c�ʌ����Ă����i���F�j
		LEDG = ON;
		LEDR = ON;
	}
	else{									// �c�ʖ��Ȃ��i�ΐF�j
		LEDG = ON;
		LEDR = OFF;
	}
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�̃��t�@�����X�l�i��̒l�j��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@�F�@�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.07			sato			�V�K
//		v1.1		2018.08.25			sato			Ref�̎擾�����m�ɂł��Ă��Ȃ��悤�Ɍ��������ߋ�����Ref��ݒ肵�Ă���i�ً}�[�u�j
// *************************************************************************/
PUBLIC void GYRO_SetRef( void )
{
	USHORT i;
	ULONG ul_ref = 0;
	
	/* �f�[�^�T���v�����O */
	for( i=0; i<GYRO_REF_NUM; i++){			// 100��T���v�����O�������ϒl����̒l�Ƃ���B
		ul_ref += (ULONG)s_GyroVal;
		TIME_wait(1);
	}
	
	/* ��l�Z�o�i���ϒl�j */
	l_GyroRef = ul_ref / GYRO_REF_NUM * 100;		// ���x��100�{�ɂ���
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   �@�\		�F �W���C���̊p���x�Ɋւ��鐧��΍����擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26		�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_GyroVal * 100 ;				// 100�{�̐��x�ɂ���
	LONG  l_err = l_val - l_GyroRef ;
	FLOAT f_res;
	
	/* �p���x�̕΍��Z�o */
	if( ( l_err < -20 * 100 ) || ( 20 * 100 < l_err ) ){
		f_res = (FLOAT)l_err /32.768 / 100;		//32.768 = 2^16(16bit)/2000(+-1000�x) LSB/(��/s)
													// 100�{�̐��x
	}
	else{
		f_res = 0;									// [deg/s]
	}
	
	return f_res;
}

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p�x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_GyroNowAngle;
}

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p�x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getRef( void )
{
	return l_GyroRef;
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�p�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
//		v2.0		2018.08.16			sato		SPI�ɂ��W���C���擾�ݒ�
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	FLOAT f_speed;
	
	/* �o�b�t�@�V�t�g�i[7]�ɐV�����f�[�^�����邽�߁A[0]�̃f�[�^���̂ĂāA1���l�߂�j */
/*	s_GyroValBuf[0]	= s_GyroValBuf[1];
	s_GyroValBuf[1]	= s_GyroValBuf[2];
	s_GyroValBuf[2]	= s_GyroValBuf[3];
	s_GyroValBuf[3]	= s_GyroValBuf[4];
	s_GyroValBuf[4]	= s_GyroValBuf[5];
	s_GyroValBuf[5]	= s_GyroValBuf[6];
	s_GyroValBuf[6]	= s_GyroValBuf[7];
*/	
	/* �ŐV�̃W���C���Z���T�l���擾 */
//	s_GyroValBuf[7] = (SHORT)recv_spi_gyro();
	
	/* �W���C���̒l�𕽊�����i��������8�j */
	s_GyroVal = (SHORT)recv_spi_gyro();//( s_GyroValBuf[0] + s_GyroValBuf[1] + s_GyroValBuf[2] + s_GyroValBuf[3] +
				//  s_GyroValBuf[4] + s_GyroValBuf[5] + s_GyroValBuf[6] + s_GyroValBuf[7] ) / 8;
	
	/* ���݂̊p�x���X�V���� */
	f_speed = GYRO_getSpeedErr();			// �p���x�擾 (0.001sec���̊p���x)
	f_GyroNowAngle += f_speed / 1000;		// �p�x�ݒ�   (0.001sec���ɉ��Z���邽��)

	/* �G���[�`�F�b�N */
	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// �p�x�ݒ�   (0.001sec���ɉ��Z���邽��)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
			Failsafe_flag();
//			printf("fail\n\r");
		}
	}
}

// *************************************************************************
//   �@�\		�F �����x�̃��t�@�����X�l�i��̒l�j��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@�F�@�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.14			sato			�V�K
// *************************************************************************/
PUBLIC void ACCEL_SetRef( void )
{
	USHORT i;
	LONG ul_ref = 0;
	
	/* �f�[�^�T���v�����O */
	for( i=0; i<ACCEL_REF_NUM; i++){			// 100��T���v�����O�������ϒl����̒l�Ƃ���B
		ul_ref += (LONG)s_AccelVal;
		TIME_wait(1);
	}
	
	/* ��l�Z�o�i���ϒl�j */
	l_AccelRef = ul_ref / ACCEL_REF_NUM ;		
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   �@�\		�F �����x�̒l���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.13		sato			�V�K
// *************************************************************************/
PUBLIC FLOAT Accel_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_AccelVal ;				// 100�{�̐��x�ɂ���
	LONG  l_err = l_val - l_AccelRef ;
	FLOAT f_res;

	f_res= (FLOAT)l_err/2048*9800;
	return f_res;
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�p�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.26			�O��			�V�K
//		v2.0		2018.08.16			sato		SPI�ɂ��W���C���擾�ݒ�
// *************************************************************************/
PUBLIC void ACCEL_Pol( void )
{	
	/* �����x�̒l���擾���� */
	s_AccelVal = (SHORT)recv_spi_accel();
	
	/* ���݂̉����x���X�V���� */
	f_NowAccel = Accel_getSpeedErr();			// �p���x�擾 (0.001sec���̉����x)

	/* �G���[�`�F�b�N */
/*	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// �p�x�ݒ�   (0.001sec���ɉ��Z���邽��)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
			Failsafe_flag();
//			printf("fail\n\r");
		}

	}
*/
}

// *************************************************************************
//   �@�\		�F �G���[���o�p���J�n����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10			sato			�V�K
// *************************************************************************/
PUBLIC void GYRO_staErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = TRUE;

}


// *************************************************************************
//   �@�\		�F �G���[���o�p���I������
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10			sato			�V�K
// *************************************************************************/
PUBLIC void GYRO_endErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;

}

// *************************************************************************
//   �@�\		�F SPI�֐�
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.06			sato			�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi(USHORT spi_ad)
{
	USHORT recv;
	RSPI0.SPDR.WORD.H = spi_ad;
	
	while(!RSPI0.SPSR.BIT.IDLNF);	//���M�J�n���m�F
	while(RSPI0.SPSR.BIT.IDLNF);		//RSPI0����ُ�Ԃ��m�F
		recv = RSPI0.SPDR.WORD.H ;

	return(recv);
}

// *************************************************************************
//   �@�\		�F SPI(whoami)
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.04			�O��			�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_who(void)
{
	USHORT recv;
	USHORT whoami = (0x75|0x80);
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(whoami);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F SPI_init
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.05			sato		�V�K
//		v1.1		2019.10.13			sato		acc�ǉ�
// *************************************************************************/
PUBLIC void recv_spi_init(void)
{
	USHORT recv;
	USHORT register107 = (0x6B|0x00);		//power management1
	USHORT register106 = (0x6A|0x00);
	USHORT register112 = (0x70|0x00);
	USHORT register27 = (0x1B|0x00);
	USHORT register28 = (0x1C|0x00);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x80);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register106);
	recv = recv_spi(0x01);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register112);
	recv = recv_spi(0x40);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27);
	recv = recv_spi(0x30);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);

	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register28);
	recv = recv_spi(0x18);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	/*read*/
/*
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27|0x80);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	return(recv);
*/
}

// *************************************************************************
//   �@�\		�F SPI_gyro_read
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.05			sato		�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_gyro(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x47|0x80);	//register71
	USHORT gyro_L = (0x48|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F SPI_accelerometer_read
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.13			sato		�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_accel(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT accel_H = (0x3D|0x80);	//register71
	USHORT accel_L = (0x3E|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(accel_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(accel_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F SPI_gyro_read
//   ����		�F �Ȃ�
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.25			sato		�V�K
// *************************************************************************/
PUBLIC USHORT recv_spi_gyrooffset(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x23|0x80);	//register23
	USHORT gyro_L = (0x24|0x80);	//register24
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   �@�\		�F �G���R�[�_�̃J�E���g���J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void ENC_Sta( void )
{
	ENC_R_TSTR = ON;		// �J�E���g�J�n
	ENC_L_TSTR = ON;		// �J�E���g�J�n
}


// *************************************************************************
//   �@�\		�F �G���R�[�_�̃J�E���g���~����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void ENC_Stop( void )
{
	ENC_R_TSTR = OFF;		// �J�E���g��~
	ENC_L_TSTR = OFF;		// �J�E���g��~
}


// *************************************************************************
//   �@�\		�F �G���R�[�_�̃J�E���g���N���A
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PRIVATE void ENC_clr( void )
{
	ENC_R_TCNT = ENC_RESET_VAL;
	ENC_L_TCNT = ENC_RESET_VAL;
}


// *************************************************************************
//   �@�\		�F �G���R�[�_�̃J�E���g�l�i�p���X���j���擾����
//   ����		�F �Ȃ�
//   ����		�F ���Ԓl����̍���
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void ENC_GetDiv( LONG* p_r, LONG* p_l )
{
	LONG l_cntR = (LONG)ENC_R_TCNT;
	LONG l_cntL = (LONG)ENC_L_TCNT;
	
	ENC_clr();		// �J�E���g�l���Z�b�g
	
	*p_r = l_cntR - ENC_RESET_VAL;		// �E���[�^
	*p_l = ENC_RESET_VAL - l_cntL;		// �����[�^
}


// *************************************************************************
//   �@�\		�F DCM�̉�]������CW�i���v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.08.19			sato	��]�����͌�Ń`�F�b�N	�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = OFF;				// BIN1
		DCM_R_IN2 = ON;			// BIN2
	}
	else{							// ��
		DCM_L_IN1 = ON;			// AIN1
		DCM_L_IN2 = OFF;				// AIN2
	}
}


// *************************************************************************
//   �@�\		�F DCM�̉�]������CCW�i�����v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = ON;			// BIN1
		DCM_R_IN2 = OFF;				// BIN2
	}
	else{							// ��
		DCM_L_IN1 = OFF;				// AIN1
		DCM_L_IN2 = ON;			// AIN2
	}
}


// *************************************************************************
//   �@�\		�F DCM���~����
//   ����		�F �Ȃ�
//   ����		�F PWM��HI�o�͒��ɖ{�֐������s����ƁA�s����100%�o�͏�ԂȂ邽�߁A�֐����Ńs�����N���A�iLo�j����B
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = OFF;			// BIN1
		DCM_R_IN2 = OFF;			// BIN2
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else{							// ��
		DCM_L_IN1 = OFF;			// AIN1
		DCM_L_IN2 = OFF;			// AIN2
		DCM_L_TIMER = OFF;			// �^�C�}��~
		DCM_L_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
}


// *************************************************************************
//   �@�\		�F DCM���u���[�L���O����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = ON;				// BIN1
		DCM_R_IN2 = ON;				// BIN2
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else{							// ��
		DCM_L_IN1 = ON;				// AIN1
		DCM_L_IN2 = ON;				// AIN2
		DCM_L_TIMER = OFF;			// �^�C�}��~
		DCM_L_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	    	DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
}


// *************************************************************************
//   �@�\		�F DCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* �^�C�}�X�^�[�g */
	if( en_id == DCM_R ){			// �E
		DCM_R_TIORA = 2;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIMER = ON;			// �^�C�}�J�n
	}
	else{							// ��
		DCM_L_TIORA = 2;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	    	DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_L_TIMER = ON;			// �^�C�}�J�n
	}
}


// *************************************************************************
//   �@�\		�F �SDCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// �E���[�^ON
	DCM_staMot(DCM_L);									// �����[�^ON
}


// *************************************************************************
//   �@�\		�F DCM��PWM-Duty��ݒ肷��
//   ����		�F ���荞�݊O����ݒ肷��ƁA�_�u���o�b�t�@�łȂ��Ǝ������ɂȂ�ꍇ������B
//   ����		�F ���荞�݃n���h��������s���邱�ƁBDuty0%�̏ꍇ���[�^���~������iPWM�ɂЂ����o��j
//   ����		�F ���[�^ID�A�H�H�H�H�H
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.03			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// ����
	USHORT	us_onReg;							// �ݒ肷��ON-duty
	
	/* PWM�ݒ� */
	if( en_id == DCM_R ){				// �E
	
		if( 0 == us_duty10 ){			// Duty0%�ݒ�
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER = OFF;			// �^�C�}��~
			DCM_R_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_R_GRB = 5000;			// �^�C�}�l�ύX
			DCM_R_TIORA = 6;			// TIOCA �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
		    	DCM_R_TIORB = 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_R_TIMER = ON;			// �^�C�}�J�n
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_R_GRA;		// ����
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_R_TIMER = OFF;			// �^�C�}��~
			DCM_R_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_R_GRB = us_onReg;		// onDuty
			DCM_staMot( en_id );		// ��]�J�n
		}
	}
	else{								// ��

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER = OFF;			// �^�C�}��~
			DCM_L_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_L_GRB = 5000;			// �^�C�}�l�ύX
			DCM_L_TIORA = 6;			// TIOCA �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
		    	DCM_L_TIORB = 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_L_TIMER = ON;			// �^�C�}�J�n
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_L_GRA;		// ����
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_L_TIMER = OFF;			// �^�C�}��~
			DCM_L_TCNT = 0;				// TCNT �J�E���^���N���A
			DCM_L_GRB = us_onReg;		// �^�C�}�l�ύX
			DCM_staMot( en_id );		// ��]�J�n
		}
	}
}

// *************************************************************************
//   �@�\		�F ������J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_sta( void )
{
	uc_CtrlFlag = TRUE;
}

// *************************************************************************
//   �@�\		�F ������~����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_stop( void )
{
	uc_CtrlFlag = FALSE;
	DCM_brakeMot( DCM_R );		// �u���[�L
	DCM_brakeMot( DCM_L );		// �u���[�L
}

// *************************************************************************
//   �@�\		�F ����f�[�^���N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_clrData( void )
{
	ENC_clr();								// �G���R�[�_���W���[��������
	l_CntR			= 0;						// �J�E���^�N���A
	l_CntL			= 0;						// �J�E���^�N���A
	
	/* ���ݒl */
	f_NowDist 		= 0;						// �ړ��������Z�b�g
	f_NowDistR 		= 0;
	f_NowDistL 		= 0;
	f_NowSpeed		= 0;						// [���x����]   ���݂̑��x [mm/s]			�i1[msec]���ɍX�V�����j
	f_NowAngle		= 0;						// [�p�x����]   ���݊p�x					�i1[msec]���ɍX�V�����j
	s_GyroVal		= 0;						// �W���C���l�N���A
	f_GyroNowAngle	= 0;							// �W���C���l�N���A
	
	/* �ڕW�l */
	f_TrgtSpeed		= 0;						// [���x����]   �ڕW�ړ����x [mm/s]			�i1[msec]���ɍX�V�����j
	f_TrgtDist 		= 0;						// [��������]   �ڕW�ړ�����				�i1[msec]���ɍX�V�����j
	f_TrgtAngleS	= 0;							// [�p���x����] �ڕW�p���x [rad/s]			�i1[msec]���ɍX�V�����j
	f_TrgtAngle		= 0;						// [�p�x����]   �ڕW�p�x					�i1[msec]���ɍX�V�����j
	
	/* ����f�[�^ */
	f_DistErrSum 	= 0;						// [��������]   �����ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
	f_AngleErrSum 	= 0;						// [�p�x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
	f_ErrDistBuf	= 0;						// [�ǐ���]     �����Z���T�[�G���[�l�̃o�b�t�@		�i1[msec]���ɍX�V�����j
	f_ErrAngleSBuf  = 0;
}


// *************************************************************************
//   �@�\		�F ����f�[�^���Z�b�g����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ����f�[�^
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_setData( stCTRL_DATA* p_data )
{
	/* ������@ */
	en_Type					= p_data->en_type;

	/* ���x���� */
	f_Acc 					= p_data->f_acc;
	f_BaseSpeed				= p_data->f_now;
	f_LastSpeed				= p_data->f_trgt;

	/* �������� */
	f_BaseDist 				= p_data->f_nowDist;
	f_LastDist 				= p_data->f_dist;

	/* �p���x���� */
	f_AccAngleS 			= p_data->f_accAngleS;
	f_BaseAngleS			= p_data->f_nowAngleS;
	f_LastAngleS			= p_data->f_trgtAngleS;

	/* �p�x���� */
	f_BaseAngle 			= p_data->f_nowAngle;
	f_LastAngle 			= p_data->f_angle;
	
	f_Time 					= 0;
	f_TrgtTime				= p_data->f_time;

	CTRL_sta();				// ����J�n
	
#if 0
	/* debug */
	printf(" [Type]%2d [Acc]%05.2f [BaseSpeed]%05.2f [LastSpeed]%05.2f [BaseDist]%05.2f [TrgtDist]%05.2f \n\r" ,
		en_Type,
		f_Acc,
		f_BaseSpeed,
		f_LastSpeed,
		f_BaseDist,
		f_LastDist
	);
	printf(" [AccA]%05.2f [BaseAngleS]%05.2f [LastAngleS]%05.2f [BaseAngle]%05.2f [LastAngle]%05.2f [Time]%05.4f\n\r\n\r" ,
		f_AccAngleS,
		f_BaseAngleS,
		f_LastAngleS,
		f_BaseAngle,
		f_LastAngle,
		f_TrgtTime
	);
#endif	
}

// *************************************************************************
//   �@�\		�F ����f�[�^�����݂̏�ԂɍX�V���� 
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void CTRL_refNow( void )
{
	FLOAT f_speedR		= 0;							// �E���[�^���ݑ��x [mm/s]
	FLOAT f_speedL		= 0;							// �����[�^���ݑ��x [mm/s]
	FLOAT f_r 			= F_CNT2MM(l_CntR);				// �E���[�^�̐i�񂾋��� [mm]
	FLOAT f_l 			= F_CNT2MM(l_CntL);				// �����[�^�̐i�񂾋��� [mm]

	/* ���x�X�V */
	f_speedR = f_r * 1000;								// �E���[�^���x [mm/s] ( �ړ�����[�J�E���g] * 1�p���X�̈ړ���(0.0509[mm]) * 1000(msec��sec) 
	f_speedL = f_l * 1000;								// �����[�^���x [mm/s] ( �ړ�����[�J�E���g] * 1�p���X�̈ړ���(0.0509[mm]) * 1000(msec��sec) 
	f_NowSpeed  = ( f_speedR + f_speedL ) / 2;			// �}�E�X�i�i�s�������S���j [1mm/s] 
	
	/* �����X�V */
	f_NowDistR += f_r;									// �J�E���g�X�V
	f_NowDistL += f_l;									// �J�E���g�X�V
	f_NowDist  = ( f_NowDistR + f_NowDistL ) / 2;		// ���ϒl�X�V
	
	
}


// *************************************************************************
//   �@�\		�F ����f�[�^��ڕW�l�ɍX�V����
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
//		v1.3		2017.11.12			sato	�X�����[���ǉ�
// *************************************************************************/
PUBLIC void CTRL_refTarget( void )
{
	/* ���샂�[�h�ɉ����� */
	switch( en_Type ){
	
		/* ������(���i) */
		case CTRL_ACC:
		case CTRL_SKEW_ACC:
			if( f_TrgtSpeed < f_LastSpeed ){												// �����ڕW�X�V���
				f_TrgtSpeed = f_BaseSpeed + f_Acc * f_Time;									// �ڕW���x
			}
			break;
		
		/* ������(���i) */
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
			f_TrgtSpeed = f_BaseSpeed;														// �ڕW���x
			break;
		
		/* ������(���i) */
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
			/* ���x���� �{ �ʒu���� */
			if( f_TrgtSpeed > f_LastSpeed ){												// �����ڕW�X�V���
				f_TrgtSpeed = f_BaseSpeed - f_Acc * f_Time;									// �ڕW���x
				f_TrgtDist  = f_BaseDist + ( f_BaseSpeed + f_TrgtSpeed ) * f_Time / 2;		// �ڕW����
			}
			/* �ʒu���� */
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;
			
		/* ������(���M�n����) */
		case CTRL_ACC_TRUN:
			
			/* �����v��� */
			if( ( f_LastAngle > 0 ) && ( f_TrgtAngleS < f_LastAngleS ) ){
				f_TrgtAngleS = 0 + f_AccAngleS * f_Time;									// �ڕW�p���x
			}
			/* ���v��� */
			else if( ( f_LastAngle < 0 ) && ( f_TrgtAngleS > f_LastAngleS ) ){
				f_TrgtAngleS = 0 - f_AccAngleS * f_Time;									// �ڕW�p���x
			}
			break;

		/* ������(���M�n����) */
		case CTRL_CONST_TRUN:
//			f_TrgtAngleS =f_BaseAngleS;
			break;

		/* ������(���M�n����) */
		case CTRL_DEC_TRUN:
		
			/* �����v��� */
			if( f_LastAngle > 0 ){ 
			
				/* �p���x���� �{ �p�x���� */
				if( f_TrgtAngleS > f_LastAngleS ){												// �����ڕW�X�V���
					f_TrgtAngleS = f_BaseAngleS - f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
				}
				/* �p�x���� */
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}
			/* ���v��� */
			else{
				
				/* �p���x���� �{ �p�x���� */
				if( f_TrgtAngleS < f_LastAngleS ){												// �����ڕW�X�V���
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
				}
				/* �p�x���� */
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}
			break;
			
		/* �X�����[���O�̑O�i����(�X�����[��) */
		case CTRL_ENTRY_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;								// �ڕW����
			}
			break;

		/* ������(�X�����[��) */
		case CTRL_ACC_SURA:
			f_TrgtSpeed = f_BaseSpeed;

			/* �����v��� */
			if( f_LastAngle > 0 ){ 
				/* �����v��� */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}
			else{
				/* ���v��� */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
//					printf("%5.2f %5.2f %5.4f %5.2f\n\r",f_TrgtAngleS,f_AccAngleS,f_Time,f_TrgtAngle);
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}

			/* �ʒu���� */
			if( f_LastDist > f_TrgtDist ){													// �ڕW�X�V���
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// �ڕW�ʒu
			}
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;

		/* ������(�X�����[��) */
		case CTRL_CONST_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = f_BaseAngleS;							// �ڕW�p���x

			/* �����v��� */
			if( f_LastAngle > 0 ){ 
				/* �����v��� */
				if( f_TrgtAngle < f_LastAngle ){
					f_TrgtAngle  = f_BaseAngle + f_TrgtAngleS * f_Time;			// �ڕW�p�x
				}
				else{
					f_TrgtAngle  = f_LastAngle;									// �ڕW�p�x
				}
			}
			else{
				/* ���v��� */
				if( f_TrgtAngle > f_LastAngle ){
					f_TrgtAngle  = f_BaseAngle + f_TrgtAngleS * f_Time;			// �ڕW�p�x
				}
				else{
					f_TrgtAngle  = f_LastAngle;									// �ڕW�p�x
				}
			}

			/* �ʒu���� */
			if( f_LastDist > f_TrgtDist ){													// �ڕW�X�V���
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// �ڕW�ʒu
			}
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;

		/* ������(�X�����[��) */
		case CTRL_DEC_SURA:
			f_TrgtSpeed = f_BaseSpeed;

			/* �����v��� */
			if( f_LastAngle > 0 ){ 
				/* �����v��� */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}
			else{
				/* ���v��� */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// �ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// �ڕW�p�x
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// �ڕW����
				}
			}
			
			/* ���x���� �{ �ʒu���� */
			if( f_LastDist > f_TrgtDist ){													// �ڕW�X�V���
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// �ڕW�ʒu
			}
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;
			
		/* �X�����[����̑O�i����(�X�����[��) */
		case CTRL_EXIT_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = 0;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;								// �ڕW����
			}
			else{
				f_TrgtDist  = f_LastDist;														// �ڕW����
			}
			break;
		
		/* ��L�ȊO�̃R�}���h */
		default:
			break;
	}
}

// *************************************************************************
//   �@�\		�F �����������p�����[�^ID�ɕϊ�����
//   ����		�F 
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04		sato			�V�K
// *************************************************************************/
PRIVATE enPARAM_MODE Chg_ParamID( enCTRL_TYPE en_type )
{
	switch( en_type ){
		case CTRL_ACC:			return PARAM_ACC;				// ������(���i)
		case CTRL_CONST:		return PARAM_CONST;				// ������(���i)
		case CTRL_DEC:			return PARAM_DEC;				// ������(���i)
		case CTRL_HIT_WALL:		return PARAM_HIT_WALL;			// �ǂ��Đ���
//		case DCMC_BACK_ACC:		return PARAM_BACK_ACC;			// ������(��i)
//		case DCMC_BACK_CONST:		return PARAM_BACK_CONST;		// ������(��i)
//		case DCMC_BACK_DEC:		return PARAM_BACK_DEC;			// ������(��i)
		case CTRL_SKEW_ACC:		return PARAM_SKEW_ACC;			// ������(���i)
		case CTRL_SKEW_CONST:		return PARAM_SKEW_CONST;		// ������(���i)
		case CTRL_SKEW_DEC:		return PARAM_SKEW_DEC;			// ������(���i)
		case CTRL_ACC_TRUN:		return PARAM_ACC_TRUN;			// ������(���n�M����)
		case CTRL_CONST_TRUN:		return PARAM_CONST_TRUN;		// ������(���n�M����)
		case CTRL_DEC_TRUN:		return PARAM_DEC_TRUN;			// ������(���n�M����)
		case CTRL_ENTRY_SURA:		return PARAM_ENTRY_SURA;		// �X�����[���O�̑O�i����(�X�����[��)
		case CTRL_ACC_SURA:		return PARAM_ACC_SURA;			// ������(�X�����[��)
		case CTRL_CONST_SURA:		return PARAM_CONST_SURA;		// ������(�X�����[��)
		case CTRL_DEC_SURA:		return PARAM_DEC_SURA;			// ������(�X�����[��)
		case CTRL_EXIT_SURA:		return PARAM_EXIT_SURA;			// �X�����[����̑O�i����(�X�����[��)
		default:			return PARAM_NC;
	}
}


// *************************************************************************
//   �@�\		�F �t�B�[�h�t�H�A�[�h�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   �@�̂̎��ʂƉ����x����K�v��OnDuty�̃I�t�Z�b�g�����Z�ł���悤�ɂ���B
//   ����		�F [�o��] �t�B�[�h�t�H�A�[�h�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getFF_speed( FLOAT* p_err )
{
	FLOAT f_ff_speed_acc = 0.0f;
	FLOAT f_ff_speed = 0.0f;

	f_ff_speed_acc	= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_speed_acc;
	f_ff_speed		= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_speed;

	/* ���샂�[�h�ɉ����� */
	switch( en_Type ){
	
		// ���� 
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_TRUN:
		case CTRL_ACC_SURA:
			*p_err = f_Acc * f_ff_speed_acc + f_TrgtSpeed * f_ff_speed ;
			break;
			
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_TRUN:
		case CTRL_ENTRY_SURA:
		case CTRL_EXIT_SURA:
		case CTRL_CONST_SURA:
			*p_err = f_TrgtSpeed * f_ff_speed ;
			break;
		
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_TRUN:
		case CTRL_DEC_SURA:
			*p_err = f_Acc * f_ff_speed_acc * (-1) + f_TrgtSpeed * f_ff_speed;
			break;

		// �����ȊO 
		default:
			*p_err = 0;
			break;										// �������Ȃ�
	}
	
}

// *************************************************************************
//   �@�\		�F �t�B�[�h�t�H�A�[�h�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   �@�̂̎��ʂƉ����x����K�v��OnDuty�̃I�t�Z�b�g�����Z�ł���悤�ɂ���B
//   ����		�F [�o��] �t�B�[�h�t�H�A�[�h�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getFF_angle( FLOAT* p_err )
{
	FLOAT f_ff_angleS_acc = 0.0f;
	FLOAT f_ff_angleS = 0.0f;

	f_ff_angleS_acc = PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_angleS_acc;
	f_ff_angleS 	= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_angleS;

	/* ���샂�[�h�ɉ����� */
	switch( en_Type ){
	
		// ���� 
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_TRUN:
		case CTRL_ACC_SURA:
			*p_err =FABS(f_AccAngleS) * f_ff_angleS_acc + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
			
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_TRUN:
		case CTRL_ENTRY_SURA:
		case CTRL_EXIT_SURA:
		case CTRL_CONST_SURA:
			*p_err = FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
		
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_TRUN:
		case CTRL_DEC_SURA:
			*p_err = FABS(f_AccAngleS) * f_ff_angleS_acc *(-1) + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		// �����ȊO 
		default:
			*p_err = 0;
			break;										// �������Ȃ�
	}
	
}

// *************************************************************************
//   �@�\		�F ���x�t�B�[�h�o�b�N�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   P������s��
//   ����		�F [�o��] ���x�t�B�[�h�o�b�N�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getSpeedFB( FLOAT* p_err )
{
	FLOAT		f_speedErr;					// [���x����] ���x�΍�
	FLOAT		f_kp = 0.0f;
	FLOAT		f_ki = 0.0f;
	FLOAT		f_kd = 0.0f;
	/* ���x���� */
	f_speedErr  = f_TrgtSpeed - f_NowSpeed;					// ���x�΍�[mm/s]
	f_kp = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_ki;
	f_kd = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_kd;
	
	/* I�������Z */
	f_SpeedErrSum += f_speedErr * f_ki;			// I�����X�V
	if( f_SpeedErrSum > 120 ){
		f_SpeedErrSum = 120;			// ������~�b�^�[
	}
	
	/* PID���� */
	*p_err = f_speedErr * f_kp + f_SpeedErrSum + ( f_speedErr - f_ErrSpeedBuf ) * f_kd;				// PI����ʎZ�o
	
	f_ErrSpeedBuf = f_speedErr;		// �΍����o�b�t�@�����O	
	
	/* �ݐϕ΍��N���A */
	if( FABS( f_speedErr ) < 0.5 ){
		f_SpeedErrSum = 0;
	}
	
}


// *************************************************************************
//   �@�\		�F �����t�B�[�h�o�b�N�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   PI������s��
//   ����		�F [�o��] �����t�B�[�h�o�b�N�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getDistFB( FLOAT* p_err )
{
	FLOAT				f_distErr;					// [��������] �����΍�
//	PRIVATE FLOAT		f_limTime = 0;				// �O�a��Ԉێ�����[sec]
	FLOAT 				f_kp = 0.0f;				// ���Q�C��
	FLOAT 				f_ki = 0.0f;				// �ϕ��Q�C��
	
	*p_err = 0;

	/* ����/�����̈ʒu���� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST )||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ))
	{
		// �Ȃɂ����Ȃ�
	}
	/* �����݈̂ʒu���� */
	else if(( en_Type == CTRL_DEC )|| ( en_Type == CTRL_SKEW_DEC ) ||
			 ( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ||
			 ( en_Type == CTRL_ACC_SURA ) || ( en_Type == CTRL_CONST_SURA ) || ( en_Type == CTRL_DEC_SURA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_ki;
		
		/* �ʒu���� */
		f_distErr  = f_TrgtDist - f_NowDist;					// �����΍�[mm]

		/* I�������Z */
		f_DistErrSum += f_distErr * f_ki;			// I�����X�V
		if( f_DistErrSum > 100 ){
			f_DistErrSum = 100;			// ������~�b�^�[
		}
		
		/* PI���� */
		*p_err = f_distErr * f_kp + f_DistErrSum;				// PI����ʎZ�o
		
		/* �ݐϕ΍��N���A */
		if( FABS( f_TrgtDist - f_NowDist ) < 0.05 ){
			f_DistErrSum = 0;
		}
	}

/* ���M�n���� */
	else if( ( en_Type == CTRL_ACC_TRUN ) || ( en_Type == CTRL_CONST_TRUN ) || ( en_Type == CTRL_DEC_TRUN ) ){
		f_distErr  = f_TrgtDist - f_NowDist;					// �����΍�[mm]
		*p_err = f_distErr * 0.1;								// P����ʎZ�o
	}

}


// *************************************************************************
//   �@�\		�F �p���x�t�B�[�h�o�b�N�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   P������s��
//   ����		�F [�o��] �p���x�t�B�[�h�o�b�N�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getAngleSpeedFB( FLOAT* p_err )
{
	FLOAT f_err;					// [����] �W���C���Z���T�[�G���[�l
	FLOAT f_kp = 0.0f;				// ���Q�C��
	FLOAT f_ki = 0.0f;				
	FLOAT f_kd = 0.0f;
	
	
	f_err = f_TrgtAngleS - GYRO_getSpeedErr();			// �ڕW�p�x - �W���C���Z���T[deg/s]
	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_ki;
	f_kd = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_kd;
	
	f_AngleSErrSum += f_err*f_ki;
	
	if(f_AngleSErrSum > 100){
		f_AngleSErrSum = 100;			//������~�b�^�[
	}
	else if(f_AngleSErrSum <-100){
		f_AngleSErrSum = -100;
	}
	
	templog2 = f_AngleSErrSum;
	*p_err = f_err * f_kp + f_AngleSErrSum + ( f_err - f_ErrAngleSBuf ) * f_kd;		// PID����
		
	f_ErrAngleSBuf = f_err;		// �΍����o�b�t�@�����O	
	
	// �ݐϕ΍��N���A 
	if( FABS( f_err ) < 1 ){
		f_AngleSErrSum = 0;
	}

	// �ݐϕ΍��N���A 
//	if( en_Type == CTRL_DEC_SURA ){
//		f_AngleSErrSum = 0;
//	}
}


// *************************************************************************
//   �@�\		�F �p�x�t�B�[�h�o�b�N�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   P������s��
//   ����		�F [�o��] �p�x�t�B�[�h�o�b�N�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
//      v1.1        2017.4.20     ����(I����ǉ�)
// *************************************************************************/
PUBLIC void CTRL_getAngleFB( FLOAT* p_err )
{
	FLOAT f_err;					// [����] �p�x�΍�[deg]
	FLOAT f_kp = 0.0f;				// ���Q�C��
	FLOAT f_ki = 0.0f;				// �ϕ��Q�C��
	
	*p_err = 0;

	f_NowAngle = GYRO_getNowAngle();					// ���݊p�x[deg]

	f_err = f_TrgtAngle - f_NowAngle;
	/* ���i�� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC )|| 
		( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA )||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_ki;
		
		f_AngleErrSum += f_err*f_ki;	//I�����X�V
		if(f_AngleErrSum > 200){
			f_AngleErrSum = 200;			//������~�b�^�[
		}
		else if(f_AngleErrSum <-200){
			f_AngleErrSum = -200;
		}
		
		//*p_err = f_err * FB_ANG_KP_GAIN;					// P����ʎZ�o
		*p_err = f_err * f_kp + f_AngleErrSum;					// PI����ʎZ�o
//		templog2 = f_AngleErrSum;

		/* �ݐϕ΍��N���A */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.3 ){
			f_AngleErrSum = 0;
		}
		
	}
	
	/* ���M�n���񎞌��� */
	else if(( en_Type == CTRL_DEC_TRUN )||
			 ( en_Type == CTRL_ACC_SURA ) || ( en_Type == CTRL_CONST_SURA ) || ( en_Type == CTRL_DEC_SURA ))
	{
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_ki;
		
		f_AngleErrSum += f_err*f_ki;	//I�����X�V
		if(f_AngleErrSum > 200){
			f_AngleErrSum = 200;			//������~�b�^�[
		}
		else if(f_AngleErrSum <-200){
			f_AngleErrSum = -200;
		}
		
		//*p_err = f_err * FB_ANG_KP_GAIN;					// P����ʎZ�o
		*p_err = f_err * f_kp + f_AngleErrSum;					// PI����ʎZ�o
//		templog2 = f_AngleErrSum;

		/* �ݐϕ΍��N���A */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.3 ){
			f_AngleErrSum = 0;
		}
	}

}


// *************************************************************************
//   �@�\		�F �ǐ���̃t�B�[�h�o�b�N�ʂ��擾����B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   PD������s��
//   ����		�F [�o��] �ǐ���̃t�B�[�h�o�b�N�����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_getSenFB( FLOAT* p_err )
{
	FLOAT f_err 	= 0;
	FLOAT f_kp 		= 0.0f;				// ���Q�C��
	FLOAT f_kd 		= 0.0f;				// �����Q�C��
	FLOAT gyro		= 0.0f;
	
	/* ���i�� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC )|| 
			 ( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_wall_kp;
		f_kd = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_wall_kd;
		
		/* �΍��擾 */
		DIST_getErr( &l_WallErr );
		f_err = (FLOAT)l_WallErr;
//		templog2 = f_err;
		/* PD���� */
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD����
		
		f_ErrDistBuf = f_err;		// �΍����o�b�t�@�����O
	}
	else if( ( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC ) ){
		
		DIST_getErrSkew( &l_WallErr );
		f_err = (FLOAT)l_WallErr;
		
//		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD����
		*p_err = f_err;
	}

}


// *************************************************************************
//   �@�\		�F FF/FB����ʂ���DCM�ɏo�͂���B
//   ����		�F CTRL_pol����̂ݎ��s�\�B
//   ����		�F 1msec���Ɏ��s�����B
//   			   ���݂̃o�b�e���d���ɍ��킹��ON-Duty�����Z���ďo�͂���B
//   ����		�F �E���[�^��Duty��A�����[�^��Duty��
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void CTRL_outMot( FLOAT f_duty10_R, FLOAT f_duty10_L )
{
	FLOAT	f_temp;			// �v�Z�p
	
	f_Duty_R = f_duty10_R;
	f_Duty_L = f_duty10_L;
	
	/* �d���ɉ�����PWM�o�͂�ύX���� */
	f_duty10_R = f_duty10_R * VCC_MAX / (BAT_getLv()/1000);
	f_duty10_L = f_duty10_L * VCC_MAX / (BAT_getLv()/1000);
	
//	f_Duty_R = f_duty10_R;
//	f_Duty_L = f_duty10_L;
	
//	log_in(f_duty10_R);
	/* �E���[�^ */
	if( 20 < f_duty10_R ){									// �O�i
		DCM_setDirCw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_duty10_R );
	}
	else if( f_duty10_R < -20 ){							// ���
		f_temp = f_duty10_R * -1;
		DCM_setDirCcw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_R );								// �u���[�L
	}

	/* �����[�^ */
	if( 20 < f_duty10_L ){									// �O�i
		DCM_setDirCcw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_duty10_L );
	}
	else if( f_duty10_L < -20 ){							// ���
		f_temp = f_duty10_L * -1;
		DCM_setDirCw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_L );								// �u���[�L
	}
}


// *************************************************************************
//   �@�\		�F ����̃|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����B1msec���Ɋ��荞�ݏ������s���B
//				�F ���DCM�̃t�B�[�h�o�b�N�ƃt�B�[�h�t�H���[�h���䏈�����s���B
//				�F ����́A���i�����̐���i���x/�����j�Ɖ�]�����̐���i�p���x/�p�x/�����Z���T�j�̂Q�����{����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void CTRL_pol( void )
{
	FLOAT f_feedFoard_speed		= 0;		// [����] �t�B�[�h�t�H���[�h����
	FLOAT f_feedFoard_angle		= 0;
	FLOAT f_speedCtrl			= 0;		// [����] ���x�����
	FLOAT f_distCtrl			= 0;		// [����] ���������
	FLOAT f_angleSpeedCtrl			= 0;		// [����] �p���x�����
	FLOAT f_angleCtrl			= 0;		// [����] �p�x�����
	FLOAT f_distSenCtrl			= 0;		// [����] �����Z���T�[�����
	FLOAT f_duty10_R;						// [�o��] �E���[�^PWM-DUTY��[0.1%]
	FLOAT f_duty10_L;						// [�o��] �����[�^PWM-DUTY��[0.1%]
	
	/* ������s�����̃`�F�b�N */
	if( uc_CtrlFlag != TRUE ){
		 return;		// ���䖳�����
	}
	if(SW_ON == SW_INC_PIN){
		Failsafe_flag();
	}
	
	/* ����s�\ */
	if( SYS_isOutOfCtrl() == TRUE ){

		f_DistErrSum = 0;				// �ݐϕ΍��N���A
		f_NowDist = f_LastDist;			// �����I�ɍŏI�ڕW�ʒu�ɕύX
		f_NowAngle = f_LastAngle;		// �����I�ɍŏI�ڕW���ԂɕύX
		f_Time = f_TrgtTime;			// �����I�ɍŏI�ڕW���ԂɕύX
		
	 	CTRL_stop();				// �����~
		CTRL_clrData();					// �f�[�^�N���A
		DCM_brakeMot( DCM_R );			// �u���[�L
		DCM_brakeMot( DCM_L );			// �u���[�L
	}
	
	/* �e��Z���T���� */
	ENC_GetDiv( &l_CntR, &l_CntL );					// �ړ���[�J�E���g�l]���擾
	CTRL_refNow();									// ����Ɏg�p����l�����݂̏�ԂɍX�V
	CTRL_refTarget();								// ����Ɏg�p����l��ڕW�l�ɍX�V

	/* ����l�擾 */
	CTRL_getFF_speed( &f_feedFoard_speed );					// [����] �t�B�[�h�t�H���[�h
	CTRL_getFF_angle( &f_feedFoard_angle );					// [����] �t�B�[�h�t�H���[�h
	CTRL_getSpeedFB( &f_speedCtrl );				// [����] ���x
	CTRL_getDistFB( &f_distCtrl );					// [����] ����
	CTRL_getAngleSpeedFB( &f_angleSpeedCtrl );			// [����] �p���x
	CTRL_getAngleFB( &f_angleCtrl );				// [����] �p�x
	CTRL_getSenFB( &f_distSenCtrl );				// [����] ��
	
	templog1 = f_angleSpeedCtrl;
//	templog1 = f_distSenCtrl;
	
	/* ���i���� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) ||( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		f_duty10_R = f_feedFoard_speed * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// �E���[�^PWM-DUTY��[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// �����[�^PWM-DUTY��[0.1%]
	}
	
	/* �ǂ��Đ��� */
	else if( en_Type == CTRL_HIT_WALL ){
		f_duty10_R = f_feedFoard_speed * FF_HIT_BALANCE_R * (-1);																		// �E���[�^PWM-DUTY��[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_HIT_BALANCE_L * (-1);
	}
	
	/* �X�����[������ */
	else if( ( en_Type == CTRL_ACC_SURA ) || (en_Type == CTRL_CONST_SURA)||( en_Type == CTRL_DEC_SURA ) ){
		/* ������ */
		if( f_LastAngle > 0 ){
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �����[�^PWM-DUTY��[0.1%]
		}
		/*�E���� */
		else{
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �����[�^PWM-DUTY��[0.1%]
		}
	}
		
	
	/* ���M�n���� */
	else{
		/* ������ */
		if( f_LastAngle > 0 ){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R        + f_angleCtrl + f_angleSpeedCtrl;									// �E���[�^PWM-DUTY��[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl;									// �����[�^PWM-DUTY��[0.1%]
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R        + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �����[�^PWM-DUTY��[0.1%]
		}
		/* �E���� */
		else{
//			f_duty10_R = f_feedFoard * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl;									// �E���[�^PWM-DUTY��[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L        - f_angleCtrl - f_angleSpeedCtrl;									// �����[�^PWM-DUTY��[0.1%]
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L        - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// �����[�^PWM-DUTY��[0.1%]
		}
	}
	
	CTRL_outMot( f_duty10_R, f_duty10_L );				// ���[�^�֏o��

	f_Time += 0.001;
	
	/* �ǐ؂�`�F�b�N */
	if( MOT_getWallEdgeType() == MOT_WALL_EDGE_RIGHT ){
		
		/* �ǔ��� */
		if( DIST_isWall_R_SIDE() == FALSE ){
			
			MOT_setWallEdge( TRUE );		// �ǂ̐؂�ڂ����m
		}
	}
	else if( MOT_getWallEdgeType() == MOT_WALL_EDGE_LEFT ){
		
		/* �ǔ��� */
		if( DIST_isWall_L_SIDE() == FALSE ){
			
			MOT_setWallEdge( TRUE );		// �ǂ̐؂�ڂ����m
		}
	}
//	log_write(GYRO_getSpeedErr(),f_NowAngle,CTRL_getAngleSpeedFB,f_duty10_R);
//	log_write(GYRO_getSpeedErr());
}


// *************************************************************************
//   �@�\		�F �����x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc1( void )
{
	return PARAM_getSpeed( PARAM_ST )->f_acc;
}


// *************************************************************************
//   �@�\		�F �����x���擾����
//   ����		�F �v���X�Ŏw��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc3( void )
{
	return PARAM_getSpeed( PARAM_ST )->f_dec;
}


// *************************************************************************
//   �@�\		�F ���i����@
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_goBlock_AccConstDec( FLOAT f_fin, enMOT_ST_TYPE en_type, enMOT_GO_ST_TYPE en_goType )
{

	stCTRL_DATA		st_data;					// ����f�[�^
//	printf("�ڕW���x %f �ڕW�ʒu %f\r\n",st_Info.f_trgt,st_Info.f_dist);
	GYRO_staErrChkAngle();
	
	/* ================ */
	/*      ������      */
	/* ================ */
	/* ------ */
	/*  ����  */
	/* ------ */
	if( ( en_type != MOT_CONST_DEC ) && ( en_type != MOT_CONST_DEC_CUSTOM ) ){

		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_ACC;
		}
		else{
			st_data.en_type		= CTRL_SKEW_ACC;
		}
		st_data.f_acc			= st_Info.f_acc1;		// �����x�w��
		st_data.f_now			= st_Info.f_now;		// ���ݑ��x
		st_data.f_trgt			= st_Info.f_trgt;		// �ڕW���x
		st_data.f_nowDist		= 0;				// �i��ł��Ȃ�
		st_data.f_dist			= st_Info.f_l1;			// ��������
		st_data.f_accAngleS		= 0;				// �p�����x
		st_data.f_nowAngleS		= 0;				// ���݊p���x
		st_data.f_trgtAngleS		= 0;				// �ڕW�p�x
		st_data.f_nowAngle		= 0;				// ���݊p�x
		st_data.f_angle			= 0;				// �ڕW�p�x
		st_data.f_time 			= 0;				// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_clrData();								// �ݒ�f�[�^���N���A
		CTRL_setData( &st_data );						// �f�[�^�Z�b�g
//		printf("�ڕW���x %f �ڕW�ʒu %f \r\n",st_data.f_trgt,st_data.f_dist);
		DCM_staMotAll();							// ���[�^ON
		while( f_NowDist < st_Info.f_l1 ){					// �w�苗�����B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
			MOT_setWallEdgeDist();
		}
//		LED4 = 0x01;
//		printf("���݈ʒu %f \r\n",f_NowDist);
	}
	
	/* ------ */
	/*  ����  */
	/* ------ */
	if( MOT_GO_ST_NORMAL == en_goType ){
		st_data.en_type		= CTRL_CONST;
	}
	else{
		st_data.en_type		= CTRL_SKEW_CONST;
	}
	st_data.f_acc			= 0;					// �����x�w��
	st_data.f_now			= st_Info.f_trgt;			// ���ݑ��x
	st_data.f_trgt			= st_Info.f_trgt;			// �ڕW���x
	st_data.f_nowDist		= st_Info.f_l1;				// ���݈ʒu
	st_data.f_dist			= st_Info.f_l1_2;			// ���������ʒu
	st_data.f_accAngleS		= 0;					// �p�����x
	st_data.f_nowAngleS		= 0;					// ���݊p���x
	st_data.f_trgtAngleS		= 0;					// �ڕW�p�x
	st_data.f_nowAngle		= 0;					// ���݊p�x
	st_data.f_angle			= 0;					// �ڕW�p�x
	st_data.f_time 			= 0;					// �ڕW���� [sec] �� �w�肵�Ȃ�
	if( ( en_type == MOT_CONST_DEC ) || ( en_type == MOT_CONST_DEC_CUSTOM ) ){
		CTRL_clrData();										// �ݒ�f�[�^���N���A
	}
	CTRL_setData( &st_data );						// �f�[�^�Z�b�g
//	printf("�ڕW���x %f �ڕW�ʒu %f \r\n",st_data.f_trgt,st_data.f_dist);
	while( f_NowDist < st_Info.f_l1_2 ){				// �w�苗�����B�҂�
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// �u���[�L
			DCM_brakeMot( DCM_L );		// �u���[�L
			break;
		}				// �r���Ő���s�\�ɂȂ���
		MOT_setWallEdgeDist();
	}
//	printf("���݈ʒu %f \r\n",f_NowDist);
//	LED4 = 0x02;

	/* ------ */
	/*  ����  */
	/* ------ */
	if( ( en_type != MOT_ACC_CONST ) && ( en_type != MOT_ACC_CONST_CUSTOM ) ){

		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_DEC;
		}
		else{
			st_data.en_type		= CTRL_SKEW_DEC;
		}
		st_data.f_acc			= st_Info.f_acc3;			// ����
		st_data.f_now			= st_Info.f_trgt;			// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;			// �ŏI���x
		st_data.f_nowDist		= st_Info.f_l1_2;			// ���������ʒu
		st_data.f_dist			= st_Info.f_dist;			// �S�ړ������ʒu
		st_data.f_accAngleS		= 0;						// �p�����x
		st_data.f_nowAngleS		= 0;						// ���݊p���x
		st_data.f_trgtAngleS		= 0;						// �ڕW�p�x
		st_data.f_nowAngle		= 0;						// ���݊p�x
		st_data.f_angle			= 0;						// �ڕW�p�x
		st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_setData( &st_data );							// �f�[�^�Z�b�g
//		printf("�ڕW���x %f �ڕW�ʒu %f \r\n",st_data.f_trgt,st_data.f_dist);
		while( f_NowDist < ( st_Info.f_dist - 0.2 ) ){		// �w�苗�����B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
			MOT_setWallEdgeDist();
		}
//		printf("���݈ʒu %f \r\n",f_NowDist);
//		LED4 = 0x04;
	}
	
	/* -------------------- */
	/*  �����i�ǂ̐؂�ځj  */
	/* -------------------- */
	/* �ǐ؂ꂪ�܂�������Ȃ���ԁi�ǐ؂�ݒ�����Ă���̂ɁA�G�b�W�������Ă��Ȃ��j */
	if( ( en_WallEdge != MOT_WALL_EDGE_NONE ) && ( bl_IsWallEdge == FALSE )  ){
	
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;						// �����x�w��
		st_data.f_now			= st_Info.f_last;			// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;			// �ڕW���x
		st_data.f_nowDist		= f_NowDist;				// ���݈ʒu
		st_data.f_dist			= f_NowDist + 180.0f;		// ���������ʒu�i180.0f�F�ǐ؂���ǂ��܂ŋ~�����̋����j�A�����ł�f_NowDist���N���A���Ă͂����Ȃ��B
		st_data.f_accAngleS		= 0;						// �p�����x
		st_data.f_nowAngleS		= 0;						// ���݊p���x
		st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
		st_data.f_nowAngle		= 0;						// ���݊p�x
		st_data.f_angle			= 0;						// �ڕW�p�x
		st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
		CTRL_setData( &st_data );							// �f�[�^�Z�b�g
		while( f_NowDist < st_data.f_dist ){				// �w�苗�����B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
			if( MOT_setWallEdgeDist_LoopWait() == TRUE ) break;	// �ǐ؂�␳�����s���鋗����ݒ�
		}
	}
	/* �ǐ؂�܂Œ��i������s�� */
	if( ( MOT_GO_ST_NORMAL == en_goType ) &&				// ���i���ɒǉ����삪�K�v�ȏꍇ�ɂ������{���Ȃ�
		( f_WallEdgeAddDist != 0.0f ) &&
		( f_fin != 0.0f )
	){
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;						// �����x�w��
		st_data.f_now			= st_Info.f_last;			// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;			// �ڕW���x
		st_data.f_nowDist		= 0;						// ���݈ʒu
		st_data.f_dist			= f_WallEdgeAddDist;		// ���������ʒu
		st_data.f_accAngleS		= 0;						// �p�����x
		st_data.f_nowAngleS		= 0;						// ���݊p���x
		st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
		st_data.f_nowAngle		= 0;						// ���݊p�x
		st_data.f_angle			= 0;						// �ڕW�p�x
		st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
		CTRL_setData( &st_data );							// �f�[�^�Z�b�g
		while( f_NowDist < st_data.f_dist ){				// �w�苗�����B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
	
	/* ��~ */
	if( 0.0f == f_fin ){
		TIME_wait(100);			// ����҂�
	 	CTRL_stop();				// �����~
		DCM_brakeMot( DCM_R );		// �u���[�L
		DCM_brakeMot( DCM_L );		// �u���[�L
	}
	
	f_MotNowSpeed = f_fin;			// ���ݑ��x�X�V
	GYRO_endErrChkAngle();
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1();								// �����x1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3();								// �����x3[mm/s^2]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;								// ���ݑ��x	
	st_Info.f_trgt		= f_MotTrgtSpeed;								// �ڕW���x
	st_Info.f_last		= f_fin;									// �ŏI���x
	
	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// �ړ�����[mm]
	st_Info.f_l1		= ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	f_l3			= ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]

//	printf("1 %f,%f\r",st_Info.f_trgt,st_Info.f_l1);
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`�����i�����j�j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1();								// �����x1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3();								// �����x3[mm/s^2]


	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// �ړ�����[mm]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;												// ���ݑ��x	
	st_Info.f_last		= f_fin;													// �ŏI���x
	st_Info.f_trgt		= sqrt( 1 / ( ( st_Info.f_acc3 * -1 ) - st_Info.f_acc1 ) *					
					( 2 * st_Info.f_acc1 * ( st_Info.f_acc3 * -1 ) * ( st_Info.f_dist - MOT_MOVE_ST_MIN ) + 
					( st_Info.f_acc3 * -1 ) * f_MotNowSpeed * f_MotNowSpeed - st_Info.f_acc1 * f_fin * f_fin ) );

	st_Info.f_l1		= ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	f_l3			= ( f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) / ( ( st_Info.f_acc3  * -1 ) * 2 );			// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]

//	printf("2 %f,%f,%f,%f\r",st_Info.f_trgt,st_Info.f_l1,f_fin,f_MotNowSpeed);
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1();													// �����x1[mm/s^2]
	st_Info.f_acc3 		= 0;																// �����x3[mm/s^2](���g�p)

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// �ړ�����[mm]
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����i�����j�j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f );	// �����x1[mm/s^2]�i�����I�ɏ��������j
	st_Info.f_acc3 		= 0;																// �����x3[mm/s^2](���g�p)

	/* ���� */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= 0;																// �����x1[mm/s^2](���g�p)
	st_Info.f_acc3 		= MOT_getAcc3();													// �����x3[mm/s^2]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// �ړ�����[mm]
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��1-2�ړ�����[mm]
}


// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����i�����j�{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;									// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;									// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;									// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= 0;																// �����x1[mm/s^2](���g�p)
	st_Info.f_acc3 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f ) * -1;	// �����x3[mm/s^2]�i�����I�ɏ��������j

	/* ���� */
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��1-2�ړ�����[mm]
}


// *************************************************************************
//   �@�\		�F �O�i�̃^�C�v���擾����
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE enMOT_ST_TYPE MOT_getStType( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT f_v1Div;
	FLOAT f_v3Div;
	FLOAT f_acc1;
	FLOAT f_acc3;
	FLOAT f_t1;
	FLOAT f_t3;
	FLOAT f_l1;							//��������
	FLOAT f_l3;							//��������
	FLOAT f_total;							// �ړ�����[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_total	= f_num * BLOCK;
	}
	else{									// �΂߂̒��i
		f_total	= f_num * BLOCK_SKEW;
	}
	

	/* ================ */
	/*  �����{��������  */
	/* ================ */
	f_v1Div		= f_fin - f_MotNowSpeed;
	f_acc1		= MOT_getAcc1();				// �����x1[mm/s^2]
	f_t1		= f_v1Div / f_acc1;

	f_l1 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t1;

	/* �����{�������� */
	if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

		/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
		if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
//			printf("�p�^�[��4\n\r");
			return MOT_ACC_CONST_CUSTOM;		// �p�^�[��4�i�����I�ɉ����x��ύX����j
		}
		else{
//			printf("�p�^�[��3\n\r");
			return MOT_ACC_CONST;				// �p�^�[��3�i�����{�����j
		}
	}

	/* ================ */
	/*  �����{��������  */
	/* ================ */
	f_v3Div		= f_fin - f_MotNowSpeed;
	f_acc3		= MOT_getAcc3();				// �����x3[mm/s^2]
	f_t3		= f_v3Div / ( f_acc3 * -1 );

	f_l3 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t3;

	/* �����{�������� */
	if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

		/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
		if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
//			printf("�p�^�[��6\n\r");
			return MOT_CONST_DEC_CUSTOM;		// �p�^�[��6�i�����I�ɉ����x��ύX����j
		}
		else{
//			printf("�p�^�[��5\n\r");
			return MOT_CONST_DEC;				// �p�^�[��5�i�����{�����j
		}
	}

	/* ========== */
	/*  ��`����  */
	/* ========== */
	f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;					// ��`���̑��x��
	f_t1		= f_v1Div / f_acc1;
	f_l1		= ( f_MotNowSpeed + f_MotTrgtSpeed ) * 0.5f * f_t1;

	f_v3Div		= f_fin - f_MotTrgtSpeed;							// ��`���̑��x��
	f_acc3		= MOT_getAcc3();									// �����x3[mm/s^2]
	f_t3		= -1.0f * f_v3Div / f_acc3;							// �������̏��v����
	f_l3		= ( f_MotTrgtSpeed + f_fin ) * 0.5f * f_t3;

	/* �ʏ�̑�`���� */
	if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){
//		printf("�p�^�[��1\n\r");
		return MOT_ACC_CONST_DEC;				// �p�^�[��1�i�ʏ�j
	}
	/* �����l��ύX���� */
	else{
//		printf("�p�^�[��2\n\r");
		return MOT_ACC_CONST_DEC_CUSTOM;		// �p�^�[��2�i�ڕW���x��ύX�j
	}
}


// *************************************************************************
//   �@�\		�F ���O�i
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PRIVATE void MOT_go_FinSpeed( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_goStType )
{
	enMOT_ST_TYPE 		en_type 		= MOT_getStType( f_num, f_fin, en_goStType);			// ����p�^�[���擾

	/* �ړ������Ǝw��l�ɉ����œ����ς��� */
	switch( en_type ){
	
		case MOT_ACC_CONST_DEC:				// [01] ��`����
			MOT_setData_ACC_CONST_DEC( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_DEC_CUSTOM:		// [02] ��`�����i�����j
			MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );		// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST:				// [03] �����{����
			MOT_setData_MOT_ACC_CONST( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_CUSTOM:		// [04] �����{�����i�����j
			MOT_setData_MOT_ACC_CONST_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, MOT_GO_ST_NORMAL );			// ����
			break;

		case MOT_CONST_DEC:				// [05] �����{����
			MOT_setData_MOT_CONST_DEC( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_CONST_DEC_CUSTOM:		// [06] �����{�����i�����l�ύX�j
			MOT_setData_MOT_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		default:
			break;
	}

#if 0
	printf(" \n\r [type]%d [acc1]%06.2f [acc3]%06.2f [dist]%06.2f [l1]%06.2f [l1_2]%06.2f [l_last]%06.2f [now]%06.2f [trgt]%06.2f \n\r", 
		en_type,
		st_Info.f_acc1,
		st_Info.f_acc3,
		st_Info.f_dist,
		st_Info.f_l1,
		st_Info.f_l1_2,
		st_Info.f_last,
		st_Info.f_now,
		st_Info.f_trgt
	);
#endif
}


// *************************************************************************
//   �@�\		�F ���O�i�i�ʏ�j
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B��
//   ����		�F ��搔�A�ŏI���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_NORMAL );		// �ʏ�̒��i
}

// *************************************************************************
//   �@�\		�F ���O�i�i�΂߁j
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B��
//   ����		�F ��搔�A�ŏI���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SKEW );		// �ʏ�̒��i
}

// *************************************************************************
//   �@�\		�F �e�X�g���s�p�v���O����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.3.28			��			�V�K
// *************************************************************************/
PUBLIC void testrun(void)
{
	stCTRL_DATA test;
		test.en_type=CTRL_ACC;
		test.f_acc=MOT_getAcc1();
		test.f_trgt	= 0;
		test.f_now = 0;
		test.f_nowDist = 0;
		test.f_dist = 0;

		
		
		
	CTRL_clrData();
	CTRL_setData(&test);
	
	
}

// *************************************************************************
//   �@�\		�F �W���C���e�X�g�v���O����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.4.18			��			�V�K
// *************************************************************************/
PUBLIC float GYRO_test(void)
{
	return s_GyroVal;
}


// *************************************************************************
//   �@�\		�F ���s�����v���O����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.6.1			��			�V�K
// *************************************************************************/
PUBLIC float dist_check(void)
{
	return f_NowDist;

}

/****************************************************************************//*!
//   �@�\		�F �}�E�X�̊p�����x1���擾����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
 *//**************************    ��    ��    *******************************//*!
 *  @date      2012.09.05    �O��      �V�K       
 *******************************************************************************/
PUBLIC FLOAT MOT_getAccAngle1( void )
{
//	return ( 1800 );
	return PARAM_getSpeed( PARAM_TRUN )->f_accAngle;
}


/****************************************************************************//*!
//   �@�\		�F �}�E�X�̊p�����x2���擾����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
 *//**************************    ��    ��    *******************************//*!
 *  @date       2012.09.05    �O��      �V�K       
 *******************************************************************************/
PUBLIC FLOAT MOT_getAccAngle3( void )
{
//	return ( 1800 );
	return PARAM_getSpeed( PARAM_TRUN )->f_decAngle;
}



// *************************************************************************
//   �@�\		�F ���M�n����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.8.14		��	�V�K
//		v1.1		2017.09.19			��ꋗ����ύX
// *************************************************************************/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type )
{
	stMOT_DATA	st_info;	//�V�[�P���X�f�[�^
	stCTRL_DATA	st_data;	//����f�[�^
//	FLOAT		f_angle2 = A2_MIN;	//�Œ��2�ړ��p�x[rad]
	FLOAT		f_angle1;	//��1�ړ��p�x[rad]
	FLOAT		f_angle3;	//��3�ړ��p�x[rad]
	FLOAT		us_trgtAngleS;	//�ڕW�p�x[rad/s]
	
	us_trgtAngleS = 300;
	
	/* ---------------- */
	/*  ����f�[�^�v�Z  */
	/* ---------------- */
	/* �����x */
	st_info.f_accAngleS1= MOT_getAccAngle1();												// �p�����x1[rad/s^2]
	st_info.f_accAngleS3= MOT_getAccAngle3();												// �p�����x3[rad/s^2]

	/* �p���x */
	st_info.f_nowAngleS	= 0;																// ���݊p���x
	st_info.f_trgtAngleS= (FLOAT)us_trgtAngleS;												// �ڕW�p���x
	st_info.f_lastAngleS= 0;																// �ŏI�p���x

	/* �p�x */
	switch( en_type ){
		case MOT_R90:	st_info.f_angle =  -90 - ANGLE_OFFSET1_R;	break;					// ��]�p�x[rad]
		case MOT_L90:	st_info.f_angle =   90 + ANGLE_OFFSET1;		break;					// ��]�p�x[rad]
		case MOT_R180:	st_info.f_angle = -180 - ANGLE_OFFSET2_R;	break;					// ��]�p�x[rad]
		case MOT_L180:	st_info.f_angle =  180 + ANGLE_OFFSET2;		break;					// ��]�p�x[rad]
		case MOT_R360:	st_info.f_angle = -360 - ANGLE_OFFSET3;		break;					// ��]�p�x[rad]
		case MOT_L360:	st_info.f_angle =  360 + ANGLE_OFFSET3;		break;					// ��]�p�x[rad]
	}
	f_angle3 = ( st_info.f_trgtAngleS - st_info.f_lastAngleS ) / 2 * ( st_info.f_trgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;						// ��3�ړ��p�x[rad]
	f_angle1 = ( 0 - st_info.f_trgtAngleS) / 2 * ( 0 - st_info.f_trgtAngleS ) / st_info.f_accAngleS1;
	
	
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -����
		st_info.f_trgtAngleS*= -1;															// ��]�������t�ɂ���
		f_angle1			*= -1;
//		f_angle2 			*= -1;															// ��]�������t�ɂ���
		f_angle3 			*= -1;															// ��]�������t�ɂ���
		st_info.f_angle1	= f_angle1;						// ��1�ړ��p�x[rad]
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;									// ��1+2�ړ��p�x[rad]
		
		/* �ŏ��ړ��������㏑�� */
		if( st_info.f_angle1 > ( A1_MIN * -1 ) ){
			st_info.f_angle1 = A1_MIN * -1;
		}
	}
	else{
		st_info.f_angle1	= f_angle1;						// ��1�ړ��p�x[rad]
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;									// ��1+2�ړ��p�x[rad]
		
		/* �ŏ��ړ��������㏑�� */
		if( st_info.f_angle1 < A1_MIN ){
			st_info.f_angle1 = A1_MIN;
		}
	}


	GYRO_staErrChkAngle();			// �G���[���o�J�n
//	printf("�ڕW�p�x %f %f %f\r\n",st_info.f_angle,st_info.f_angle1,st_info.f_angle1_2);
	/* ================ */
	/*      ������      */
	/* ================ */
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_ACC_TRUN;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= 0;						// ���ݑ��x
	st_data.f_trgt			= 0;						// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= 0;						// ��������
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS		= st_info.f_trgtAngleS;		// �ڕW�p�x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= st_info.f_angle1;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	DCM_staMotAll();									// ���[�^ON
	
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -����
		while( f_NowAngle > st_info.f_angle1 ){			// �w��p�x���B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS );
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
	else{
		while( f_NowAngle < st_info.f_angle1 ){			// �w��p�x���B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
//	printf("finish\n");
//	LED4 = 0x01;
//	log_in(100);
	/* ------ */
	/*  ����  */
	/* ------ */
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -����
		f_angle3			= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// ��3�ړ��p�x[rad]
		f_angle3			= -1 * f_angle3;
		if( f_angle3 > A3_MIN*-1 ) f_angle3 = A3_MIN * -1;																	// �����Œ�p�x�ɏ�������
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;// ��1+2�ړ��p�x[rad]
		
	}
	else{
		f_angle3			= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// ��3�ړ��p�x[rad]
		if( f_angle3 < A3_MIN ) f_angle3 = A3_MIN;																			// �����Œ�p�x�ɏ�������
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;																// ��1+2�ړ��p�x[rad]
//		printf("   [f_angle3]%d [f_angle1_2]%d\n\r", (LONG)f_angle3, (LONG)	st_info.f_angle1_2 );
	}
//	printf("[f_TrgtAngleS] %5.2f,st_info.f_angle1_2%5.2f,f_angle2%5.2f\n\r",f_TrgtAngleS,st_info.f_angle1_2,f_angle3);
	st_data.en_type			= CTRL_CONST_TRUN;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= 0;						// ���ݑ��x
	st_data.f_trgt			= 0;						// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= 0;						// ���������ʒu
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;				// ���݊p���x
	st_data.f_trgtAngleS		= f_TrgtAngleS;				// �ڕW�p�x
	st_data.f_nowAngle		= st_info.f_angle1;			// ���݊p�x
	st_data.f_angle			= st_info.f_angle1_2;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -����
		while( f_NowAngle > st_info.f_angle1_2 ){			// �w�苗�����B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
//			log_in(f_TrgtAngle);
		}
	}
	else{
		while( f_NowAngle < st_info.f_angle1_2 ){			// �w�苗�����B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
//			log_in(f_TrgtAngleS);
		}
	}
//	printf("finish2\n");
//	LED4 = 0x02;
//	log_in(100);
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_DEC_TRUN;
	st_data.f_acc			= 0;						// ����
	st_data.f_now			= 0;						// ���ݑ��x
	st_data.f_trgt			= 0;						// �ŏI���x
	st_data.f_nowDist		= 0;						// ���������ʒu
	st_data.f_dist			= 0;						// �S�ړ������ʒu
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// �p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;				// ���݊p���x
	st_data.f_trgtAngleS		= 0;						// �ڕW�p�x
	st_data.f_nowAngle		= st_info.f_angle1_2;		// ���݊p�x
	st_data.f_angle			= st_info.f_angle;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -����
		while( f_NowAngle > ( st_info.f_angle + 1 ) ){		// �w�苗�����B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS );
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
	else{
		while( f_NowAngle < ( st_info.f_angle - 1 ) ){		// �w�苗�����B�҂�
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
//			log_in(f_TrgtAngle);
		}
	}
//	printf("finish3\n");
//	LED4 = 0x04;
//	log_in(200);
	/* ��~ */
	TIME_wait(500);				// ����҂�
	CTRL_stop();			// �����~
	DCM_brakeMot( DCM_R );		// �u���[�L
	DCM_brakeMot( DCM_L );		// �u���[�L
	GYRO_endErrChkAngle();					// �G���[���o�I��
	

	
}

// *************************************************************************
//   �@�\		�F �X�����[���̊J�n���x��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato			�V�K
// *************************************************************************/
PUBLIC void MOT_setSuraStaSpeed( FLOAT f_speed )
{
	f_MotSuraStaSpeed = f_speed;
	
}

// *************************************************************************
//   �@�\		�F �X�����[���̊J�n���x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getSuraStaSpeed( void )
{
	return f_MotSuraStaSpeed;
}


// *************************************************************************
//   �@�\		�F �ڕW���x��ݒ肷��B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_setTrgtSpeed(FLOAT f_speed)
{
	f_MotTrgtSpeed = f_speed;
	return f_MotTrgtSpeed;
}

// *************************************************************************
//   �@�\		�F ���ݑ��x��ݒ肷��B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato			�V�K
// *************************************************************************/
PUBLIC void MOT_setNowSpeed(FLOAT f_speed)
{
	f_MotNowSpeed = f_speed;
}

// *************************************************************************
//   �@�\		�F �Ǔ���
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.10.12		��	�V�K
// *************************************************************************/
PUBLIC void MOT_goHitBackWall(void)
{
	stMOT_DATA	st_info;	//�V�[�P���X�f�[�^
	stCTRL_DATA	st_data;	//����f�[�^

	/* ---------------- */
	/*  ����f�[�^�v�Z  */
	/* ---------------- */
	/* �����x */
	st_info.f_acc1= 1200;												// �p�����x1[rad/s^2]												// �p�����x3[rad/s^2]

	GYRO_staErrChkAngle();			// �G���[���o�J�n
//	printf("");
	/* ================ */
	/*      ������      */
	/* ================ */
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_HIT_WALL;
	st_data.f_acc			= st_info.f_acc1;						// �����x�w��
	st_data.f_now			= 0;						// ���ݑ��x
	st_data.f_trgt			= 0;						// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= 0;						// ��������
	st_data.f_accAngleS		= 0;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS		= 0;		// �ڕW�p�x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= 0;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	DCM_staMotAll();									// ���[�^ON
//	printf("�ڕW���x %f �ڕW�ʒu %f\r\n",st_data.f_trgt,st_data.f_dist);
//	LED4 =0x01;
	/*��~*/
	TIME_wait(700);				// ����҂�
	CTRL_stop();			// �����~
	DCM_brakeMot( DCM_R );		// �u���[�L
	DCM_brakeMot( DCM_L );		// �u���[�L
	
	TIME_wait(100);
	
	f_MotNowSpeed = 0.0f;		//���ݑ��x�X�V
//	LED4 = 0x08;
	GYRO_endErrChkAngle();					// �G���[���o�I��
	
}

// *************************************************************************
//   �@�\		�F �X�����[��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.11.11		��	�V�K
// *************************************************************************/
PUBLIC void MOT_goSla( enMOT_SURA_CMD en_type, stSLA* p_sla )
{
	stMOT_DATA		st_info;					// �V�[�P���X�f�[�^
	stCTRL_DATA		st_data;					// ����f�[�^
//	FLOAT			f_err;
	FLOAT			f_entryLen;
	FLOAT			f_escapeLen;
	
	/* ---------------- */
	/*  ����f�[�^�v�Z  */
	/* ---------------- */
	/* �����x */
	st_info.f_acc1 		= 0;																// �����x1[mm/s^2]
	st_info.f_acc3 		= 0;																// �����x3[mm/s^2]

	/* ���x */
	st_info.f_now		= p_sla->f_speed;													// ���ݑ��x	
	st_info.f_trgt		= p_sla->f_speed;													// �ڕW���x
	st_info.f_last		= p_sla->f_speed;													// �ŏI���x
	
	/* ���� */
	st_info.f_dist		= 0;																// �ړ�����
	st_info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_info.f_l1_2		= 0;																// ��1+2�ړ�����[mm]

	/* �p�����x */
	st_info.f_accAngleS1= p_sla->f_angAcc;													// �p�����x1[deg/s^2]
	st_info.f_accAngleS3= p_sla->f_angAcc;													// �p�����x3[deg/s^2]

	/* �p���x */
	st_info.f_nowAngleS	= 0;																// ���݊p���x[deg/s]
	st_info.f_trgtAngleS= p_sla->f_angvel;													// �ڕW�p���x
	st_info.f_lastAngleS= 0;																// �ŏI�p���x

	/* �p�x */
	st_info.f_angle		= p_sla->f_ang_Total;												// ����p�x[deg]
	st_info.f_angle1	= p_sla->f_ang_AccEnd;												// ��1�ړ��p�x[deg]
	st_info.f_angle1_2	= p_sla->f_ang_ConstEnd;											// ��1+2�ړ��p�x[deg]

	/* �����ɉ����ĕ�����ύX */
	if( ( en_type == MOT_R90S ) || 
		( en_type == MOT_R45S_S2N ) || ( en_type == MOT_R45S_N2S ) || 
		( en_type == MOT_R90S_N ) || 
		( en_type == MOT_R135S_S2N ) || ( en_type == MOT_R135S_N2S )
	){
		st_info.f_accAngleS1 *= -1;
		st_info.f_trgtAngleS *= -1;
		st_info.f_angle      *= -1;
		st_info.f_angle1     *= -1;
		st_info.f_angle1_2   *= -1;
	}
	else{
		st_info.f_accAngleS3 *= -1;
	}
	
	/* �΂ߑ��s�̃^�C�v�ɉ����āA�X�����[���O�̋����ƃX�����[����̑ޔ����������ւ��� */
	if( ( en_type == MOT_R45S_N2S ) || ( en_type == MOT_L45S_N2S ) || ( en_type == MOT_R135S_N2S ) || ( en_type == MOT_L135S_N2S ) ){ 		// �t�ɂ������
		f_entryLen  = p_sla->f_escapeLen;
		f_escapeLen = p_sla->f_entryLen;
	}
	else{		// �ʏ�
		f_entryLen  = p_sla->f_entryLen;
		f_escapeLen = p_sla->f_escapeLen;
	}

	GYRO_staErrChkAngle();			// �G���[���o�J�n
	
//	LED_on(LED1);
	/* ================ */
	/*      ������      */
	/* ================ */
	/* ------------------------ */
	/*  �X�����[���O�̑O�i����  */
	/* ------------------------ */
	st_data.en_type			= CTRL_ENTRY_SURA;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= f_entryLen;				// �X�����[���O�̑O�i����
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= 0;						// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	DCM_staMotAll();									// ���[�^ON

	while( f_NowDist < f_entryLen ){				// �w�苗�����B�҂�
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// �u���[�L
			DCM_brakeMot( DCM_L );		// �u���[�L
			break;
		}				// �r���Ő���s�\�ɂȂ���
	}
//	LED_off(LED1);
//	log_in(0);
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_ACC_SURA;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen;				// 
	st_data.f_dist			= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;		// ��������
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS		= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= st_info.f_angle1;			// �ڕW�p�x
	st_data.f_time 			= p_sla->us_accAngvelTime * 0.001;			// [msec] �� [sec]
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
//	printf("trgtangleS %5.2f\n\r",st_data.f_trgtAngleS);
	if( IS_R_SLA( en_type ) == TRUE ) {		// -����
		while( ( f_NowAngle > st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){			// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){			// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}

//	log_in(0);
//	log_in(f_NowAngle);
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_CONST_SURA;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime ) * 0.001;		// ��������
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// ���݊p���x
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1;			// ���݊p�x
	st_data.f_angle			= st_info.f_angle1_2;		// �ڕW�p�x
	st_data.f_time 			= p_sla->us_constAngvelTime * 0.001;		// [msec] �� [sec]
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g

	if( IS_R_SLA( en_type ) == TRUE ) {		// -����
		while( ( f_NowAngle > st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){		// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){		// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
		}
	}
//	log_in(0);
//	log_in(f_NowAngle);
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_DEC_SURA;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime ) * 0.001;
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;		// ��������
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// �p�����x
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// ���݊p���x
	st_data.f_trgtAngleS		= 0;				// �ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1_2;		// ���݊p�x
	st_data.f_angle			= st_info.f_angle;			// �ڕW�p�x
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;			// [msec] �� [sec]
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
//	log_in(st_data.f_angle);
	if( IS_R_SLA( en_type ) == TRUE ) {		// -����
		while( ( f_NowAngle > st_info.f_angle + 0.2 ) || ( f_NowDist < st_data.f_dist ) ){			// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
//		LED4 = LED4_ALL_ON;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle - 0.2) || ( f_NowDist < st_data.f_dist ) ){			// �w��p�x�{�������B�҂�
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
//		LED4 = LED4_ALL_ON;
		}
	}
//	LED4 = LED4_ALL_OFF;
//	log_in(0);
//	LED_on(LED1);
	/* ------------------------ */
	/*  �X�����[����̑O�i����  */
	/* ------------------------ */
	st_data.en_type			= CTRL_EXIT_SURA;
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;
	st_data.f_dist			= f_escapeLen + f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;	// �X�����[����̑O�i����
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS		= 0;						// �ڕW�p�x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= 0;						// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	
	while( f_NowDist < ( st_data.f_dist - 0.01 ) ){	// �w�苗�����B�҂�
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// �u���[�L
			DCM_brakeMot( DCM_L );		// �u���[�L
			break;
		}				// �r���Ő���s�\�ɂȂ���
	}
//	LED_off(LED1);
//	log_in(f_NowAngle);
	f_MotNowSpeed = st_info.f_now;			// ���ݑ��x�X�V

	GYRO_endErrChkAngle();					// �G���[���o�I��

}

// *************************************************************************
//   �@�\		�F �^�[���e�[�u��(����|)
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.4.23		��	�V�K
// *************************************************************************/
PUBLIC void turntable()
{
	stCTRL_DATA test;
		test.en_type = CTRL_CONST;
		test.f_acc			= 0;						// �����x�w��
		test.f_now			= 0;			// ���ݑ��x
		test.f_trgt			= 0;			// �ڕW���x
		test.f_nowDist			= 0;
		test.f_dist			= 0;	
		test.f_accAngleS		= 0;						// �p�����x
		test.f_nowAngleS		= GYRO_getSpeedErr();						// ���݊p���x
		test.f_trgtAngleS		= 0;						// �ڕW�p�x
		test.f_nowAngle			= GYRO_getNowAngle();						// ���݊p�x
		test.f_angle			= 0;						// �ڕW�p�x
		test.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
		
	CTRL_clrData();
	CTRL_setData(&test);
	DCM_staMotAll();									// ���[�^ON
	while( 1 ){	// �w�苗�����B�҂�
		if ( SW_ON == SW_INC_PIN ){
			CTRL_stop();
			break;
		}
	}
}

// *************************************************************************
//   �@�\		�F �������O�i
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.8.30		��	�V�K
// *************************************************************************/
PUBLIC void MOT_goBlock_Const(FLOAT f_num)
{
	stCTRL_DATA		st_data;
	stMOT_DATA		st_info;
	
	GYRO_staErrChkAngle();
	
	/* ---------------- */
	/*  ����f�[�^�v�Z  */
	/* ---------------- */
	/* ���� */
	st_info.f_dist		= f_num * BLOCK - f_num * sliplengs;													// �ړ�����[mm]
	
	
	/* ------ */
	/*  ����  */
	/* ------ */
	st_data.en_type			= CTRL_CONST;
	st_data.f_acc			= 0;					// �����x�w��
	st_data.f_now			= f_MotNowSpeed;			// ���ݑ��x
	st_data.f_trgt			= f_MotNowSpeed;			// �ڕW���x
	st_data.f_nowDist		= 0;				// ���݈ʒu
	st_data.f_dist			= st_info.f_dist;			// ���������ʒu
	st_data.f_accAngleS		= 0;					// �p�����x
	st_data.f_nowAngleS		= 0;					// ���݊p���x
	st_data.f_trgtAngleS		= 0;					// �ڕW�p�x
	st_data.f_nowAngle		= 0;					// ���݊p�x
	st_data.f_angle			= 0;					// �ڕW�p�x
	st_data.f_time 			= 0;					// �ڕW���� [sec] �� �w�肵�Ȃ�
	CTRL_clrData();										// �ݒ�f�[�^���N���A
	CTRL_setData( &st_data );						// �f�[�^�Z�b�g
	f_TrgtSpeed		= f_MotNowSpeed;
//	printf("�ڕW���x %f �ڕW�ʒu %f \r\n",st_data.f_trgt,st_data.f_dist);
	while( f_NowDist < st_info.f_dist ){				// �w�苗�����B�҂�
		if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}				// �r���Ő���s�\�ɂȂ���
	}
	
	GYRO_endErrChkAngle();
}

// *************************************************************************
//   �@�\		�F �ǐ؂�␳�̃^�C�v��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.9.27		��	�V�K
// *************************************************************************/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type )
{
	en_WallEdge = en_type;
	bl_IsWallEdge = FALSE;			// �񌟒m
	
}

// *************************************************************************
//   �@�\		�F �ǐ؂�␳�̃^�C�v���擾����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.9.27		��	�V�K
// *************************************************************************/
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void )
{
	return en_WallEdge;
}

// *************************************************************************
//   �@�\		�F �ǐ؂�̌��m��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.9.27		��	�V�K
// *************************************************************************/
PUBLIC void MOT_setWallEdge( BOOL bl_val )
{
	bl_IsWallEdge = bl_val;
	
}

// *************************************************************************
//   �@�\		�F �ǐ؂ꋗ��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.9.27		��	�V�K
// *************************************************************************/
PRIVATE BOOL MOT_setWallEdgeDist( void )
{
	FLOAT f_addDist;
	
	/* �ǂ̐؂�ڂ����m���Ă��Ȃ� */
	if( ( bl_IsWallEdge == FALSE ) || ( en_WallEdge == MOT_WALL_EDGE_NONE ) ){		// �ǐ؂�ݒ肳��Ă��Ȃ����A���o���Ă��Ȃ��ꍇ�͏����𔲂���
	
		return FALSE;
	}
	
	f_addDist = f_NowDist + MOT_WALL_EDGE_DIST;		// ����J�n�ʒu
	
	/* ��������K�v������ */
	if( f_addDist > st_Info.f_dist ){
		
		f_WallEdgeAddDist = f_addDist - st_Info.f_dist;
	}
	
	/* �ǂ̐؂�ڕ␳�̕ϐ��������� */
	en_WallEdge   = MOT_WALL_EDGE_NONE;		// �ǂ̐؂�ڃ^�C�v
	bl_IsWallEdge = FALSE;					// �ǂ̐؂�ڌ��m
	
	return TRUE;
}
PRIVATE BOOL MOT_setWallEdgeDist_LoopWait( void )
{
	/* �ǂ̐؂�ڂ����m���Ă��Ȃ� */
	if( bl_IsWallEdge == FALSE ){		// �ǐ؂�ݒ肳��Ă��Ȃ����A���o���Ă��Ȃ��ꍇ�͏����𔲂���
	
		return FALSE;
	}
	
	f_WallEdgeAddDist = MOT_WALL_EDGE_DIST;		// ����J�n�ʒu
	
	return TRUE;
}

// *************************************************************************
//   �@�\		�F �t�F�C���Z�[�t�t���O�I��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10		��	�V�K
// *************************************************************************/
PUBLIC void Failsafe_flag(void)
{
	bl_failsafe = TRUE;
	LED4 = LED4_ALL_ON;
}

// *************************************************************************
//   �@�\		�F �t�F�C���Z�[�t�t���O�I�t
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10		��	�V�K
// *************************************************************************/
PUBLIC void Failsafe_flag_off(void)
{
	bl_failsafe = FALSE;
}

// *************************************************************************
//   �@�\		�F �t�F�C���Z�[�t�t���O
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F 
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.10		��	�V�K
// *************************************************************************/
PUBLIC BOOL SYS_isOutOfCtrl( void )
{
	if( bl_failsafe == TRUE ){
		return TRUE;
	}
	else{
		return FALSE;
	}
}


// *************************************************************************
//   �@�\		�F ���O���֐�(�^)
//   ����		�F �Ȃ�
//   ����		�F ���O��1�`12 ���O�����ۂ�log_flag��TRUE�ɕύX
//   ����		�F ���O�����p�����[�^
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.05.01		��	�V�K
// *************************************************************************/
PRIVATE void log_in2( 	FLOAT log1,FLOAT log2,
			FLOAT log3,FLOAT log4,
			FLOAT log5,FLOAT log6,
			FLOAT log7,FLOAT log8,
			FLOAT log9,FLOAT log10,
			FLOAT log11,FLOAT log12)
{
	if((b_logflag == TRUE)&&(log_count < log_num)){
		Log_1[log_count] = log1;
		Log_2[log_count] = log2;
		Log_3[log_count] = log3;
		Log_4[log_count] = log4;
		Log_5[log_count] = log5;
		Log_6[log_count] = log6;
		Log_7[log_count] = log7;
		Log_8[log_count] = log8;
		Log_9[log_count] = log9;
		Log_10[log_count] = log10;
		Log_11[log_count] = log11;
		Log_12[log_count] = log12;
		
		log_count++;
	}
}

// *************************************************************************
//   �@�\		�F ���O���֐�(���荞�ݗp�j
//   ����		�F �Ȃ�
//   ����		�F ���O��1�`12�@�o�b�e���[�̊��荞�݂ɒǉ��@�����œ��̓f�[�^��ύX�@intprg.h�ɒ��ڏ��������10ms�Ŋ��荞��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.05.01		��	�V�K
// *************************************************************************/
PUBLIC void log_interrupt ( void )
{
/*	log_in2(f_DistErrSum, f_NowSpeed,
		f_TrgtSpeed, f_NowDist,
		f_TrgtDist, f_AccAngleS,
		GYRO_getSpeedErr(), f_TrgtAngleS,)
		f_NowAngle, f_TrgtAngle,
		templog1,templog2);
*/

	log_in2(f_NowSpeed, f_TrgtSpeed,
		f_NowDist, f_TrgtDist,
		GYRO_getSpeedErr(), f_TrgtAngleS,
		f_NowAngle,f_TrgtAngle,
		f_AccAngleS,templog1,
		templog2,f_Duty_R);

/*	log_in2(DIST_getNowVal( DIST_SEN_R_FRONT ), DIST_getNowVal( DIST_SEN_L_FRONT ),
		DIST_getNowVal( DIST_SEN_R_SIDE ), DIST_getNowVal( DIST_SEN_L_SIDE ),
		GYRO_getSpeedErr(), f_TrgtAngleS,
		f_NowAngle,f_TrgtAngle,
		templog2,templog1,
		f_Duty_L,f_Duty_R);
*/
}

// *************************************************************************
//   �@�\		�F ���O�t���O�I��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.05.01		��	�V�K
// *************************************************************************/
PUBLIC void log_flag_on(void)
{
	b_logflag = TRUE;
}

// *************************************************************************
//   �@�\		�F ���O�t���O�I��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.05.01		��	�V�K
// *************************************************************************/
PUBLIC void log_flag_off(void)
{
	b_logflag = FALSE;
}

// *************************************************************************
//   �@�\		�F ���O�����o��
//   ����		�F �Ȃ�
//   ����		�F ���O��1�`12�@CSV�ŏo�͈���
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.05.01		��	�V�K
// *************************************************************************/
PUBLIC void log_read2(void)
{
	int i=0;
	while(i<log_num){
		printf("%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n\r",
		Log_1[i],Log_2[i],Log_3[i],Log_4[i],Log_5[i],Log_6[i],Log_7[i],Log_8[i],Log_9[i],Log_10[i],Log_11[i],Log_12[i]);
		i++;
	}
}