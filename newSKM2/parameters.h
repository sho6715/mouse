//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �p�����[�^�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���}�~
#ifndef _PARAMETER_H
#define _PARAMETER_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O

//**************************************************
// ��`�idefine�j
//**************************************************
#define		SW_CHATTERING_WAIT		(200) 		//�X�C�b�`�̃`���^�����O�΍�

#define	FF_BALANCE_R				( 1.00f )					// [FF] �E�̃o�����X�W��
#define	FF_BALANCE_L				( 1.00f )					// [FF] ���̃o�����X�W�� 
#define FF_HIT_BALANCE_R			(1.00f)						//�o�b�N���̃o�����X�W��
#define FF_HIT_BALANCE_L			(1.00f)

#define SEARCH_SPEED				(500)

#define DEG_TO_RAD  (3.1416f/180.0f)
#define RAD_TO_DEG  (180.0f/3.1416f)

#define MOT_WALL_EDGE_DIST			( 45.0f )	// �ǐ؂�Z���TOFF�`�ǂ܂�

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ������@ */
typedef enum{
	
	/* ========================================== */ 
	/*  �p�����[�^���擾����ۂɎg�p����V���{��  */ 
	/* ========================================== */ 
	/* ---------- */
	/*  ���i����  */
	/* ---------- */
	PARAM_ST_TOP = 0,				// �J�E���g�p
	// �� �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ACC,					// ������(���i)
		PARAM_CONST,				// ������(���i)
		PARAM_DEC,					// ������(���i)
//		PARAM_BACK_ACC,				// ������(��i)
//		PARAM_BACK_CONST,			// ������(��i)
//		PARAM_BACK_DEC,				// ������(��i)
		PARAM_SKEW_ACC,				// ������(�΂�)
		PARAM_SKEW_CONST,			// ������(�΂�)
		PARAM_SKEW_DEC,				// ������(�΂�)
		PARAM_HIT_WALL,				// �ǂ��Đ���

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_ST_BTM,					// �J�E���g�p
	
	/* -------- */
	/*  �^�[��  */
	/* -------- */
	PARAM_TRUN_TOP,					// �J�E���g�p
	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ACC_TRUN,				// ������(���n�M����)
		PARAM_CONST_TRUN,			// ������(���n�M����)
		PARAM_DEC_TRUN,				// ������(���n�M����)

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_TRUN_BTM,					// �J�E���g�p
	
	/* ------------ */
	/*  �X�����[��  */
	/* ------------ */
	PARAM_SLA_TOP,					// �J�E���g�p
	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ENTRY_SURA,			// �X�����[���O�̑O�i����(�X�����[��)
		PARAM_ACC_SURA,				// ������(�X�����[��)
		PARAM_CONST_SURA,			// ������(�X�����[��)
		PARAM_DEC_SURA,				// ������(�X�����[��)
		PARAM_EXIT_SURA,			// �X�����[����̑O�i����(�X�����[��)

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_SLA_BTM,					// �J�E���g�p
	
	
	/* ===================================================================== */ 
	/*  PARAM_setGainType()�ɂă��[�h�����߂�ۂɈ����Ƃ��Ďg�p����V���{��  */ 
	/* ===================================================================== */ 
	PARAM_ST,						// ���i����
	PARAM_TRUN,						// ���񐧌�
	PARAM_SLA,						// �X�����[������
	
	
	/* ====================================================== */ 
	/*  �쐬����f�[�^�����J�E���g���邽�߂Ɏg�p����V���{��  */ 
	/* ====================================================== */ 
	PARAM_ST_MAX		= PARAM_ST_BTM   - PARAM_ST_TOP,		// ���i�ő吔
	PARAM_TRUN_MAX		= PARAM_TRUN_BTM - PARAM_TRUN_TOP,		// ����ő吔
	PARAM_SURA_MAX		= PARAM_SLA_BTM  - PARAM_SLA_TOP,		// �X�����[���ő吔
	
	
	PARAM_NC = 0xff,
	
}enPARAM_MODE;


/* ���쑬�x */
typedef enum{
	
	PARAM_VERY_SLOW = 0,	// ���ᑬ
	PARAM_SLOW,				// �ᑬ
	PARAM_NORMAL,			// �ʏ�
	PARAM_FAST,				// ����
	PARAM_VERY_FAST,		// ������
	
	PARAM_MOVE_SPEED_MAX
	
}enPARAM_MOVE_SPEED;


//**************************************************
// �\���́istruct�j
//**************************************************
/* ���x�f�[�^ */
typedef struct{
	FLOAT			f_acc;					// �����x�i�������j
	FLOAT			f_dec;					// �����x�i�������j
	FLOAT			f_accAngle;				// �p�����x�i�������j
	FLOAT			f_decAngle;				// �p�����x�i�������j
}stSPEED;

/* �Q�C�� */
typedef struct{
	FLOAT			f_FF;				// �t�B�[�h�t�H���[�h
	FLOAT 			f_FB_speed_kp;			// �t�B�[�h�o�b�N�A���x ��ᐧ��
	FLOAT 			f_FB_speed_ki;			// �t�B�[�h�o�b�N�A���x �ϕ�����
	FLOAT 			f_FB_speed_kd;			// �t�B�[�h�o�b�N�A���x ��������
	FLOAT			f_FB_dist_kp;			// �t�B�[�h�o�b�N�A���� ��ᐧ��
	FLOAT 			f_FB_dist_ki;			// �t�B�[�h�o�b�N�A���� �ϕ�����
	FLOAT			f_FB_angleS_kp;			// �t�B�[�h�o�b�N�A�p���x ��ᐧ��
	FLOAT			f_FB_angleS_ki;			// �t�B�[�h�o�b�N�A�p���x �ϕ�����
	FLOAT			f_FB_angleS_kd;			// �t�B�[�h�o�b�N�A�p���x ��������
	FLOAT			f_FB_angle_kp;			// �t�B�[�h�o�b�N�A�p�x ��ᐧ��
	FLOAT			f_FB_angle_ki;			// �t�B�[�h�o�b�N�A�p�x �ϕ�����
	FLOAT			f_FB_wall_kp;			// �t�B�[�h�o�b�N�A�� ��ᐧ��
	FLOAT			f_FB_wall_kd;			// �t�B�[�h�o�b�N�A�� ��������
}stGAIN;

/* �X�����[���f�[�^ */
typedef struct{
	FLOAT	f_speed;
	FLOAT	f_angAcc;
	FLOAT	f_angvel;
	FLOAT	f_entryLen;
	FLOAT	f_escapeLen;
	USHORT	us_accAngvelTime;
	USHORT	us_constAngvelTime;
	FLOAT	f_ang_AccEnd;
	FLOAT	f_ang_ConstEnd;
	FLOAT	f_ang_Total;
}stSLA;

/* �X�����[���^�C�v */
typedef enum{
	SLA_90,
	SLA_45,	
	SLA_135,
	SLA_N90,				// �΂� �� 90���� �΂�
	SLA_TYPE_MAX
}enSLA_TYPE;



//**************************************************
// �O���[�o���ϐ�
//**************************************************




//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed );
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode );
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode );

PUBLIC void PARAM_makeSra( float f_speed, float f_angAcc, float f_g , enSLA_TYPE en_mode);
PUBLIC stSLA* PARAM_getSra( enSLA_TYPE en_mode );
#endif	