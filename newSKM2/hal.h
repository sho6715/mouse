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
#ifndef _HAL_H
#define _HAL_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <parameters.h>

//**************************************************
// ��`�idefine�j
//**************************************************
/* ���H���� */
#define HALF_BLOCK			( 45.0f*2 )					// ����� [mm]
#define BLOCK				( 90.0f*2 )					// �P��� [mm]
#define HALF_BLOCK_SKEW			( 63.64f*2 )				// �΂ߔ���� [mm]
#define BLOCK_SKEW			( 127.28f*2 )				// �΂߂P��� [mm]

#define IS_R_SLA(a)			( ( (a) % 2 == 0 ) ? (TRUE) : (FALSE))

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ���[�^ID */
typedef enum{
	DCM_R = 0,					// �E���[�^
	DCM_L,						// �����[�^
	DCM_MAX
}enDCM_ID;

/* ����R�}���h���X�g */
typedef enum{
	MOT_R90 =0,					// �E 90�x���M�n����
	MOT_L90,					// �� 90�x���M�n����
	MOT_R180,					// �E180�x���M�n����
	MOT_L180,					// ��180�x���M�n����
	MOT_R360,					// �E360�x���M�n����
	MOT_L360,					// ��360�x���M�n����
	MOT_TURN_CMD_MAX
}enMOT_TURN_CMD;

/* �X�����[���R�}���h���X�g */
typedef enum{
	MOT_R90S =0,				// �E 90�x���X�����[��
	MOT_L90S,					// �� 90�x���X�����[��
	MOT_R45S_S2N,				// [�΂ߗp] �E 45�x���X�����[���A�X�g���[�g �� �΂�
	MOT_L45S_S2N,				// [�΂ߗp] �� 45�x���X�����[���A�X�g���[�g �� �΂�
	MOT_R45S_N2S,				// [�΂ߗp] �E 45�x���X�����[���A�΂� �� �X�g���[�g
	MOT_L45S_N2S,				// [�΂ߗp] �� 45�x���X�����[���A�΂� �� �X�g���[�g
	MOT_R90S_N,					// [�΂ߗp] �E 90�x���X�����[���A�΂� �� �΂�
	MOT_L90S_N,					// [�΂ߗp] �� 90�x���X�����[���A�΂� �� �΂�
	MOT_R135S_S2N,				// [�΂ߗp] �E135�x���X�����[���A�X�g���[�g �� �΂�
	MOT_L135S_S2N,				// [�΂ߗp] ��135�x���X�����[���A�X�g���[�g �� �΂�
	MOT_R135S_N2S,				// [�΂ߗp] �E135�x���X�����[���A�΂� �� �X�g���[�g
	MOT_L135S_N2S,				// [�΂ߗp] ��135�x���X�����[���A�΂� �� �X�g���[�g
	MOT_SURA_CMD_MAX,
}enMOT_SURA_CMD;

/* �ǐ؂�␳ */
typedef enum{
	MOT_WALL_EDGE_NONE =0,		// �ǂ̃G�b�W���o�ł̕␳�Ȃ�
	MOT_WALL_EDGE_RIGHT,		// �E�ǂ̃G�b�W���o�ł̕␳
	MOT_WALL_EDGE_LEFT,			// ���ǂ̃G�b�W���o�ł̕␳
	MOT_WALL_EDGE_MAX,
}enMOT_WALL_EDGE_TYPE;

//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************




//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
/*���O���*/
//PUBLIC void log_in(FLOAT log_input);
//PUBLIC void log_out(void);

PUBLIC void HAL_init( void );
/* �V���A���ʐM */
PUBLIC void SCI_putch(char data);
PUBLIC void SCI_puts(char *buffer);
PUBLIC void SCI_putsL(char *buffer, int len);
PUBLIC void charput(unsigned char data);
PUBLIC int SCI_chkRecv(void);
PUBLIC char SCI_getch(void);
PUBLIC unsigned char SCI_getch_uc(void);
PUBLIC int SCI_gets(char *buffer);
PUBLIC unsigned char charget(void);

/* �d���Ď� */
PUBLIC void BAT_Pol( void );
PUBLIC FLOAT BAT_getLv(void);

/*�W���C���Ď�*/
PUBLIC void GYRO_Pol(void);
PUBLIC void GYRO_SetRef(void);
PUBLIC FLOAT GYRO_getSpeedErr(void);
PUBLIC FLOAT GYRO_getNowAngle(void);
PUBLIC FLOAT GYRO_getRef( void );

PUBLIC void GYRO_staErrChkAngle( void );
PUBLIC void GYRO_endErrChkAngle( void );

PUBLIC void ACCEL_SetRef( void );
PUBLIC FLOAT Accel_getSpeedErr( void );
PUBLIC void ACCEL_Pol( void );


/*SPI*/
PUBLIC USHORT recv_spi(USHORT spi_ad);
PUBLIC USHORT recv_spi_who(void);
PUBLIC void recv_spi_init(void);
PUBLIC USHORT recv_spi_gyro(void);
PUBLIC USHORT recv_spi_gyrooffset(void);
PUBLIC USHORT recv_spi_accel(void);

/* �G���R�[�_ */
PUBLIC void ENC_Sta( void );
PUBLIC void ENC_Stop( void );
PUBLIC void ENC_Clr( void );
PUBLIC void ENC_GetDiv( LONG* p_r, LONG* p_l );

/* DCM */
PUBLIC void DCM_setDirCw( enDCM_ID en_id );
PUBLIC void DCM_setDirCcw( enDCM_ID en_id );
PUBLIC void DCM_stopMot( enDCM_ID en_id );
PUBLIC void DCM_brakeMot( enDCM_ID en_id );
PUBLIC void DCM_staMot( enDCM_ID en_id );
PUBLIC void DCM_staMotAll( void );
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 );

/* ���� */
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin );
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin );

/*�e�X�g���s�p*/
PUBLIC void testrun(void);

/*�W���C���e�X�g*/
PUBLIC float GYRO_test(void);

/*���M�n����*/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type );

/*�ړI���x�ύX*/
PUBLIC FLOAT MOT_setTrgtSpeed(FLOAT f_speed);
PUBLIC void MOT_setNowSpeed(FLOAT f_speed);

PUBLIC void MOT_setSuraStaSpeed( FLOAT f_speed );
PUBLIC FLOAT MOT_getSuraStaSpeed( void );

/*�Ǔ���*/
PUBLIC void MOT_goHitBackWall(void);

/*�X�����[��*/
PUBLIC void MOT_goSla( enMOT_SURA_CMD en_type, stSLA* p_sla );

/*�������O�i*/
PUBLIC void MOT_goBlock_Const(FLOAT f_num);

/*�ǐ؂�*/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type );
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void );
PUBLIC void MOT_setWallEdge( BOOL bl_val );

PRIVATE BOOL MOT_setWallEdgeDist( void );
PRIVATE BOOL MOT_setWallEdgeDist_LoopWait( void );

//�t�F�C���Z�[�t
PUBLIC void Failsafe_flag(void);
PUBLIC void Failsafe_flag_off(void);
PUBLIC BOOL SYS_isOutOfCtrl( void );

//���O�֐�
PUBLIC void log_interrupt ( void );
PUBLIC void log_flag_on(void);
PUBLIC void log_flag_off(void);
PUBLIC void log_read2(void);

#endif	// _HAL_H