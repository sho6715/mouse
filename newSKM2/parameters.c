//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �p�����[�^�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>
#include <math.h>
#include <parameters.h>
#include <hal.h>

//**************************************************
// ��`�idefine�j
//**************************************************



//**************************************************
// �񋓑́ienum�j
//**************************************************


//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �C���f�b�N�X���Z�Ɏg�p */
#define GET_INDEX_ST(i)			( i - PARAM_ST_TOP - 1 )		// ���i�p�̃C���f�b�N�X���擾
#define GET_INDEX_TRUN(i)		( i - PARAM_TRUN_TOP - 1 )		// ����p�̃C���f�b�N�X���擾
#define GET_INDEX_SLA(i)		( i - PARAM_SLA_TOP - 1 )		// �X�����[���p�̃C���f�b�N�X���擾

PRIVATE enPARAM_MOVE_SPEED		en_Speed_st	= PARAM_NORMAL;			// ���i���̈ړ����x�^�C�v
PRIVATE enPARAM_MOVE_SPEED		en_Speed_trun	= PARAM_NORMAL;				// ���񎞂̈ړ����x�^�C�v
PRIVATE enPARAM_MOVE_SPEED		en_Speed_sla	= PARAM_NORMAL;				// �X�����[�����̈ړ����x�^�C�v
PRIVATE stSLA				st_Sla[SLA_TYPE_MAX];					// �X�����[�����̑��s�p�����[�^

//********************************************************
/* ���W���[���O���[�o���ϐ��i�`���[�j���O���K�v�ȃp�����[�^�j*/
//********************************************************

/* ============ */
/*  ���x�f�[�^  */
/* ============ */

	/* ���i���x�f�[�^ */
	PRIVATE CONST stSPEED f_StSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	�����x		�����x		�p�����x		�p�����x
		{ 800,			1000,		0,				0,				},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 1800,			1800,		0,				0,				},		// �ᑬ(PARAM_SLOW)
		{ 1800,			1800,		0,				0,				},		// �ʏ�(PARAM_NORMAL)
		{ 2500,			2500,		0,				0,				},		// ����(PARAM_FAST)
		{ 4000,			4000,		0,				0,				},		// ������(PARAM_VERY_FAST)
	};

	/* ���񑬓x�f�[�^ */
	PRIVATE CONST stSPEED f_TurnSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	�����x		�����x		�p�����x		�p�����x
		{ 0,			0,			1800,			1800,			},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 0,			0,			1800,			1800,			},		// �ᑬ(PARAM_SLOW)
		{ 0,			0,			1800,			1800,			},		// �ʏ�(PARAM_NORMAL)
		{ 0,			0,			1800,			1800,			},		// ����(PARAM_FAST)
		{ 0,			0,			1800,			1800,			},		// ������(PARAM_VERY_FAST)
	};

	/* �X�����[�����x�f�[�^ */
	PRIVATE CONST stSPEED f_SlaSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	�����x		�����x		�p�����x		�p�����x
		{ 1800,			1800,		1800,			1800,			},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 1800,			1800,		1800,			1800,			},		// �ᑬ(PARAM_SLOW)
		{ 1800,			1800,		1800,			1800,			},		// �ʏ�(PARAM_NORMAL)
		{ 2500,			2500,		1800,			1800,			},		// ����(PARAM_FAST)
		{ 4000,			4000,		1800,			1800,			},		// ������(PARAM_VERY_FAST)
	};


/* ============== */
/*  �Q�C���f�[�^  */
/* ============== */
	// �y�A�h�o�C�X�z 
	//    �������Q�C���̃p�����[�^���𑝂₵�����ꍇ�́AstGAIN�̃����o�Ɓ��̃f�[�^�𑝂₷������OK�ł��B
	//    PARAM_getGain()�Ńp�����[�^�̃A�h���X���擾���āA�ǉ����������o���Q�Ƃ��ĉ������B

	/* ���i�Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_StGainData[PARAM_MOVE_SPEED_MAX][PARAM_ST_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_BACK_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_DEC
//			{ 0.0242f*4,	3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3	},	// PARAM_SKEW_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,	0,	0,		0,	0,	0,		0,		0,		0,		0,		0,		0,	},	// PARAM_HIT_WALL
		},
		/* �ᑬ(PARAM_SLOW) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_BACK_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_DEC
//			{ 0.0242f*4,	3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3	},	// PARAM_SKEW_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,	0,	0,		0,	0,	0,		0,		0,		0,		0,		0,		0,	},	// PARAM_HIT_WALL
		},
		/* �ʏ�(PARAM_NORMAL) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.01,		3,	0.03,	0,		0.1,	0,	0.2,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_BACK_ACC
			{ 0,		3,	0.03,	0,		00.1,	0,	0.2,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.2,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_DEC
//			{ 0.0242f*4,	3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		3,	0.03,	0,		0,	0,	0.2,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.2,		0.005,		0,		2,		0,		0.5,		0.3	},	// PARAM_SKEW_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.2,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,	0,	0,		0,	0,	0,		0,		0,		0,		0,		0,		0,	},	// PARAM_HIT_WALL
		},
		/* ����(PARAM_FAST) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_BACK_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_DEC
//			{ 0.0242f*4,	3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3	},	// PARAM_SKEW_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,	0,	0,		0,	0,	0,		0,		0,		0,		0,		0,		0,	},	// PARAM_HIT_WALL
		},
		/* ������(PARAM_VERY_FAST) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_BACK_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_DEC
//			{ 0.0242f*4,	3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_ACC
			{ 0,		3,	0.03,	0,		0,	0,	0.1,		0,		0,		2,		0,		0.5,		0.3	},	// PARAM_SKEW_CONST
			{ 0,		3,	0.03,	0,		6,	1,	0.1,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,	0,	0,		0,	0,	0,		0,		0,		0,		0,		0,		0,	},	// PARAM_HIT_WALL
		}
	};

	/* ����Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TRUN_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.004,	0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0.5,		0.5,		0,		0,	},	// PARAM_DEC_TRUN
		},
		/* �ᑬ(PARAM_SLOW) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.004,	0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0.5,		0.5,		0,		0,	},	// PARAM_DEC_TRUN
		},
		/* �ʏ�(PARAM_NORMAL) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.004,	0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0.5,		0.5,		0,		0,	},	// PARAM_DEC_TRUN
		},
		/* ����(PARAM_FAST) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.004,	0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0.5,		0.5,		0,		0,	},	// PARAM_DEC_TRUN
		},
		/* ������(PARAM_VERY_FAST) */
		{	//	FF	���xkp	���xki	���xkd		�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0.004,	0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0,		0,		0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0.1,	0,	0,		0.1,	0,	0.6,		0.01,		0,		0.5,		0.5,		0,		0,	},	// PARAM_DEC_TRUN
		}
	};

	/* �X�����[���Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SURA_MAX] = {

		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	//	FF	���xkp	���xki	���xkd	�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_ENTRY_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_ACC_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_CONST_SURA
			{ 0,		5,	0.03,	0,	5,	0,	3,		0.01,		0.01,		10,		0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.6,		0.4,	},	// PARAM_EXIT_SURA
		},
		/* �ᑬ(PARAM_SLOW) */
		{	//	FF	���xkp	���xki	���xkd	�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_ENTRY_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.6,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_ACC_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.6,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_CONST_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.6,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_DEC_SURA
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.6,		0.4,	},	// PARAM_EXIT_SURA
		},
		/* �ʏ�(PARAM_NORMAL)500 *///�p���xkp500 �p�xkp25000
		{	//	FF	���xkp	���xki	���xkd	�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0,		3,	0.03,	0,	0.1,	0,	0.1,		0.005,		0,		2,		0,		0.5,		0.3,	},	// PARAM_ENTRY_SURA
			{ 0,		3,	0.03,	0,	0,	0,	0.9,		0.005,		0.2,		0,		0,		0,		0,	},	// PARAM_ACC_SURA
			{ 0,		3,	0.03,	0,	0,	0,	0.9,		0.005,		0.2,		0,		0,		0,		0,	},	// PARAM_CONST_SURA
			{ 0,		3,	0.03,	0,	0,	0,	0.9,		0.005,		0.2,		0,		0,		0,		0,	},	// PARAM_DEC_SURA
			{ 0,		3,	0.03,	0,	0.1,	0,	0.1,		0.005,		0,		2,		0,		0.6,		0.4,	},	// PARAM_EXIT_SURA
		},
		/* ����(PARAM_FAST) 600*/
		{	//	FF	���xkp	���xki	���xkd	�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0,		3,	0.03,	0,	0.1,	0,	0.1,		0.01,		0,		2,		0,		0.5,		0.3,	},	// PARAM_ENTRY_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.008,		0.03,		2,		0,		0,		0,	},	// PARAM_ACC_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.008,		0.03,		2,		0,		0,		0,	},	// PARAM_CONST_SURA
			{ 0,		5,	0.03,	0,	5,	0,	2.3,		0.008,		0.03,		10,		0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0,		3,	0.03,	0,	0.1,	0,	0.1,		0.01,		0,		2,		0,		0.6,		0.4,	},	// PARAM_EXIT_SURA
		},
		/* ������(PARAM_VERY_FAST) */
		{	//	FF	���xkp	���xki	���xkd	�ʒukp	�ʒuki	�p���xkp	�p���xki	�p���xkd	�p�xkp		�p�xki		��kp		��kd
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.5,		0.3,	},	// PARAM_ENTRY_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_ACC_SURA
			{ 0,		5,	0.03,	0,	5,	0,	0.5,		0.01,		0.01,		2,		0,		0,		0,	},	// PARAM_CONST_SURA
			{ 0,		5,	0.03,	0,	5,	0,	3,		0.01,		0.01,		10,		0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0,		3,	0.03,	0,	0.1,	0,	0.2,		0,		0,		2,		0,		0.6,		0.4,	},	// PARAM_EXIT_SURA
		}
	};
	

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F ������@�ɑΉ��������쑬�x��ݒ肷��
//   ����		�F ����O�ɗ\�ߐݒ肵�Ă���
//   ����		�F ���x�l��Q�C���l���擾����ۂɁA�ǂ̓��쑬�x�̃p�����[�^���擾���邩�����肷�邽�߂Ɏg�p����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato		�V�K
// *************************************************************************/
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed )
{
	switch( en_mode ){
		
		case PARAM_ST:
			en_Speed_st = en_speed;
			break;
		
		case PARAM_TRUN:
			en_Speed_trun = en_speed;
			break;
		
		case PARAM_SLA:
			en_Speed_sla = en_speed;
			break;
			
		default:
			printf("�ݒ肵�����x�̃p�����[�^�^�C�v������܂��� \n\r");
			break;
	}
}

// *************************************************************************
//   �@�\		�F �����x�̃|�C���^���擾����
//   ����		�F 
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato		�V�K
// *************************************************************************/
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode )
{
	CONST stSPEED* p_adr;
	
	switch( en_mode ){
		
		case PARAM_ST:													// ���i
		case PARAM_ACC:													// ������(���i)
		case PARAM_CONST:												// ������(���i)
		case PARAM_DEC:													// ������(���i)
//		case PARAM_BACK_ACC:											// ������(��i)
//		case PARAM_BACK_CONST:											// ������(��i)
//		case PARAM_BACK_DEC:											// ������(��i)
		case PARAM_SKEW_ACC:											// ������(�΂�)
		case PARAM_SKEW_CONST:											// ������(�΂�)
		case PARAM_SKEW_DEC:											// ������(�΂�)
		case PARAM_HIT_WALL:											// �ǂ��Đ���
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
			
		case PARAM_TRUN:												// ����
		case PARAM_ACC_TRUN:											// ������(���n�M����)
		case PARAM_CONST_TRUN:											// ������(���n�M����)
		case PARAM_DEC_TRUN:											// ������(���n�M����)
			p_adr = &f_TurnSpeedData[en_Speed_trun];
			break;
			
		case PARAM_SLA:													// �X�����[��
		case PARAM_ENTRY_SURA:											// �X�����[���O�̑O�i����(�X�����[��)
		case PARAM_ACC_SURA:											// ������(�X�����[��)
		case PARAM_CONST_SURA:											// ������(�X�����[��)
		case PARAM_DEC_SURA:											// ������(�X�����[��)
		case PARAM_EXIT_SURA:											// �X�����[����̑O�i����(�X�����[��)
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;

		default:														// Err�A�Ƃ肠�����E�E�E�i�������j���h�����߁j
			printf("�ݒ肵�����x�^�C�v������܂��� \n\r");
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
	}
	
	return p_adr;
}


// *************************************************************************
//   �@�\		�F �Q�C���̃|�C���^���擾����
//   ����		�F 
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.04			sato		�V�K
// *************************************************************************/
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode )
{
	CONST stGAIN* p_adr;
	
	switch( en_mode ){
		
		case PARAM_ACC:													// ������(���i)
		case PARAM_CONST:												// ������(���i)
		case PARAM_DEC:													// ������(���i)
//		case PARAM_BACK_ACC:											// ������(��i)
//		case PARAM_BACK_CONST:											// ������(��i)
//		case PARAM_BACK_DEC:											// ������(��i)
		case PARAM_SKEW_ACC:											// ������(�΂�)
		case PARAM_SKEW_CONST:											// ������(�΂�)
		case PARAM_SKEW_DEC:											// ������(�΂�)
		case PARAM_HIT_WALL:											// �ǂ��Đ���
			p_adr = &f_StGainData[en_Speed_st][GET_INDEX_ST( en_mode )];
			break;
			
		case PARAM_ACC_TRUN:											// ������(���n�M����)
		case PARAM_CONST_TRUN:											// ������(���n�M����)
		case PARAM_DEC_TRUN:											// ������(���n�M����)
			p_adr = &f_TurnGainData[en_Speed_trun][GET_INDEX_TRUN( en_mode )];
			break;
			
		case PARAM_ENTRY_SURA:											// �X�����[���O�̑O�i����(�X�����[��)
		case PARAM_ACC_SURA:											// ������(�X�����[��)
		case PARAM_CONST_SURA:											// ������(�X�����[��)
		case PARAM_DEC_SURA:											// ������(�X�����[��)
		case PARAM_EXIT_SURA:											// �X�����[����̑O�i����(�X�����[��)
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
		
		default:														// Err�A�Ƃ肠�����E�E�E�i�������j���h�����߁j
			printf("�ݒ肵���Q�C���^�C�v������܂��� \n\r");
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
	}
	
	return p_adr;
}

// *************************************************************************
//   �@�\		�F �X�����[���̑��s�p�����[�^���쐬����
//   ����		�F 
//   ����		�F f_speed�F�i�����x[mm/s]�Af_angAcc�F�p�����x[deg/s^2]�Af_g�F��G[mm/s^2]�Aen_mode�F�X�����[���^�C�v
//   ����		�F ���i�����̑��x�͐i�����x�̂܂܁A��]������������������B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.18			sato		�V�K
// *************************************************************************/
PUBLIC void PARAM_makeSra( float f_speed, float f_angAcc, float f_g , enSLA_TYPE en_mode)
{

	FLOAT	f_start_x;					// �J�nx�ʒu [mm]
	FLOAT	f_start_y;					// �J�ny�ʒu [mm]
	FLOAT	f_final_x;					// �ŏIx�ʒu [mm]
	FLOAT	f_final_y;					// �ŏIy�ʒu [mm]
	FLOAT	f_final_ang;				// �p�������̍ŏI�p�x [rad]	
	FLOAT	f_maxAngleV		= 0;		// �ő�p���x[rad/s]
	FLOAT	f_timeAcc		= 0;		// ��������[s]
	FLOAT	f_accAngle		= 0;		// �����p�x[rad]
	FLOAT	f_timeConst		= 0;		// ��������[s]
	FLOAT	f_constAngle	= 0;		// �����p�x[rad]
	FLOAT	f_ang			= 0;		// ���Z�p�A�p�x [rad]
	FLOAT	f_time			= 0;		// ���Z�p�A���� [s]
	FLOAT	f_x;						// ���Z�px�ʒu [mm]
	FLOAT	f_y;						// ���Z�py�ʒu [mm]
	USHORT	i = 0;						// ���[�v�p
	stSLA* 	p_adr = &st_Sla[en_mode];		// �L�^���鑖�s�f�[�^

	/* �X�����[���ɉ������ݒ�l����X�����[���ɕK�v�ȃp�����[�^�����Z���� */
	switch(en_mode){

		case SLA_90:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK;
			f_final_y   = HALF_BLOCK;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;

		case SLA_45:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 0.75f;
			f_final_y   = BLOCK * 0.75f;
			f_final_ang = 45.0f * DEG_TO_RAD;
			break;
			
		case SLA_N90:
			f_start_x   = HALF_BLOCK * 0.5f * 1.4142f;
			f_start_y   = 0.0f;
			f_final_x   = HALF_BLOCK * 1.4142f;
			f_final_y   = HALF_BLOCK * 0.5f * 1.4142f;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;
			
		case SLA_135:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 1.25f;
			f_final_y   = BLOCK * 0.25;
			f_final_ang = 135.0f * DEG_TO_RAD;
			break;
	}

	/* �������p�x�̎Z�o */
	f_maxAngleV		= f_g / f_speed;							// �ő�p���x[rad/s] �i��[rad/s] = g[mm/s^2] / v[mm/s] �j
	f_timeAcc		= f_maxAngleV / f_angAcc;					// �ő�̊p���x�ɂȂ�܂ł̉�������[s]
	f_accAngle		= 0.5f * f_angAcc * f_timeAcc * f_timeAcc;	// �����������Ԃ̊p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
	f_constAngle	= f_final_ang - f_accAngle * 2;				// ���p���x�̋�Ԃ̊p�x[rad] (��[rad] = Total�p�x - �����p�x + �����p�x )
	f_timeConst		= f_constAngle / f_maxAngleV;				// �ő�̊p���x�œ��삷�鎞��[s]�i t[s] = ��[rad] / ��[rad/s] �j

	/* -------------------------------- */
	/*  �X�����[���������̈ʒu�����߂�  */
	/* -------------------------------- */
	/* ���W�J�n�ʒu */
	f_x		= f_start_x;
	f_y		= f_start_y;

	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	=  0.001f * (FLOAT)i;								// ����[s]
		f_ang	=  0.5f * f_angAcc * f_time * f_time;				// �p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;			// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;			// Y���W[mm]
	}
	
	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeConst*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// ����[s]
		f_ang	 = f_accAngle + f_maxAngleV * f_time;			// �p�x[rad] (��[rad] = ��[rad/s] * t[s] )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y���W[mm]
	}

	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// ����[s]
		f_ang	 = f_accAngle + f_constAngle +0.5f * f_angAcc * f_time * f_time;	// �p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y���W[mm]
	}

	/* ---------------------------- */
	/*  �X�����[���p�p�����[�^�쐬  */
	/* ---------------------------- */
	p_adr->f_speed				= f_speed;										// �i�����x[mm/s]
	printf("�i�����x %5.2f\n\r",f_speed);
	p_adr->f_angAcc				= f_angAcc * RAD_TO_DEG ;						// �p�����x[deg/s]
	printf("�p�����x %5.2f\n\r",f_angAcc * RAD_TO_DEG);
	p_adr->f_angvel				= f_maxAngleV * RAD_TO_DEG;						// �ő�p���x���Z�o �ő�p���x[deg/s]
	printf("�ő�p���x %5.2f\n\r",f_maxAngleV * RAD_TO_DEG);
	p_adr->us_accAngvelTime		= (USHORT)( f_timeAcc * 1000.0f );				// �p��������[msec]
	printf("�p�������� %5.2f\n\r",f_timeAcc * 1000.0f);
	p_adr->us_constAngvelTime	= (USHORT)( f_timeConst * 1000.0f );			// ���p������[msec]
	printf("���p������ %5.2f\n\r",f_timeConst * 1000.0f);
	p_adr->f_ang_AccEnd			= f_accAngle * RAD_TO_DEG;						// �p���������p�x[deg]
	printf("�p���� %5.2f\n\r",f_accAngle * RAD_TO_DEG);
	p_adr->f_ang_ConstEnd		= ( f_accAngle + f_constAngle ) * RAD_TO_DEG;	// ���p���x�����p�x[deg]
	printf("�p���� %5.2f\n\r",( f_accAngle + f_constAngle ) * RAD_TO_DEG);
	p_adr->f_ang_Total			= f_final_ang * RAD_TO_DEG;						// �S�ړ��p�x[deg]
	printf("�S�ړ� %5.2f\n\r",f_final_ang * RAD_TO_DEG);
	
	/* �K�v�Ȑi���Ƒޏo�̋������Z�o���� */
	switch(en_mode){
		case SLA_90:
			p_adr->f_escapeLen = f_final_x - f_x ;//-4
			p_adr->f_entryLen  = f_final_y - f_y ;//+8
			break;

		case SLA_45:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y - ( f_final_x - f_x );
			break;

		case SLA_N90:
			p_adr->f_escapeLen = f_final_x - f_x;
			p_adr->f_entryLen  = f_final_y - f_y;
			break;

		case SLA_135:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y + ( f_final_x - f_x );
			break;
	}
	printf("entry %5.2f\n\r",f_final_x - f_x);
	printf("escape %5.2f\n\r",f_final_y - f_y);
}

// *************************************************************************
//   �@�\		�F �X�����[���̑��s�p�����[�^�̊i�[��A�h���X���擾����
//   ����		�F 
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.18			sato		�V�K
// *************************************************************************/
PUBLIC stSLA* PARAM_getSra( enSLA_TYPE en_mode )
{
	return &st_Sla[en_mode];
}