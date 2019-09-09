//**********************************************************************/
//  ���{�b�g��  :newSKM(����)                                           
//  �T�v			:���C���t�@�C��                                     
//  ����			:�Ȃ�                                          
//  ����		    :�Ȃ�                                                                                        
//*********************************************************************
//   v1.0	2018.06.21		sato		�V�K
//*********************************************************************

//*********************************************************************
//   �v���W�F�N�g�������ɐ������ꂽ�R�[�h�i�g��Ȃ��j
//*********************************************************************
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif


#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>				//��`
#include <iodefine.h>				//I/O
#include <stdio.h>					//�W�����o��
#include <math.h>
#include <hal.h>					//HAL
#include <init.h>
#include <parameters.h>
//#include <search.h>
#include <search2.h>
#include <map_cmd.h>
#include <hal_dist.h>
#include <DataFlash.h>

//**************************************************
// ��`�idefine�j
//**************************************************


//**************************************************
// �񋓑́ienum�j
//**************************************************
typedef enum{
	MODE_0 = 0,					// ���[�h0 �F �o�b�e���[
	MODE_1,						// ���[�h1 �F �W���C��
	MODE_2,						// ���[�h2 �F �����Z���T
	MODE_3,						// ���[�h3 �F �G���R�[�_�̒l�����A���^�C���\��
	MODE_4,						// ���[�h4 �F ���[�^��]
	MODE_5,						// ���[�h5 �F ���[�h�̐������L�ڂ���
	MODE_6,						// ���[�h6 �F ���[�h�̐������L�ڂ���
	MODE_7,						// ���[�h7 �F 
	MODE_8,						// ���[�h8 �F ���[�h�̐������L�ڂ���
	MODE_9,
	MODE_10,
	MODE_11,
	MODE_12,
	MODE_13,
	MODE_14,
	MODE_15,
	MODE_MAX
}enMODE;

//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************
PRIVATE VUSHORT		uc_Msec;		//�������v[msec]
PRIVATE VUSHORT		uc_Sec;			//�������v[sec]
PRIVATE VUSHORT		uc_Min;			//�������v[min]
PRIVATE VULONG		ul_Wait;		//1msec��wait�Ɏg�p����J�E���^[msec]
PRIVATE enMODE		en_Mode;		//���݂̃��[�h
//SPI�p
PUBLIC USHORT recv;

float RUN_SPEED;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void INTC_sys( void );
PRIVATE void SYS_start( void );
PUBLIC void INTC_sen( void );
PUBLIC void CTRL_pol( void );

// *************************************************************************
//   �@�\		�F �}�C�R���̃^�C�}���J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.25			sato		�V�K
// *************************************************************************/
PRIVATE void RX631_staTimer(void)
{
	MTU.TSTR.BIT.CST0		= 1;//MTU0�J�E���g�J�n	�V�X�e��
	MTU.TSTR.BIT.CST1		= 1;//MTU1�J�E���g�J�n  �o�b�e���[�Ď�
	MTU.TSTR.BIT.CST3		= 1;//MTU3�J�E���g�J�n  �Z���T�Ď�
	TPUA.TSTR.BIT.CST1		= 1;//TPU1�J�E���g�J�n  �ʑ��v��
	TPUA.TSTR.BIT.CST2		= 1;//TPU2�J�E���g�J�n  �ʑ��v��
}

// *************************************************************************
//   �@�\		�F ���Ԑ���̏������B
//   ����		�F �N����A1�񂾂����s����֐��B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K
// *************************************************************************/
PUBLIC void TIME_init( void )
{
	/* ������ */
	uc_Msec = 0;		// �������v[msec]
	uc_Sec  = 0;		// �������v[sec]
	uc_Min  = 0;		// �������v[min]
	ul_Wait = 0;		// 1msec��wait�Ɏg�p���� �J�E���^[msec]
}

// *************************************************************************
//   �@�\		�F �w�肵�� ms �ԁAS/W�E�F�C�g����B
//   ����		�F �Ȃ�
//   ����		�F ul_time: �҂��� ( msec �P�� )�B �ő� 600000(= 10 min) ����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K
// *************************************************************************/
PUBLIC void TIME_wait( ULONG ul_time )
{
	ul_Wait = 0;						// 0����J�E���g����A"ul_Wait"��1msec����1���Z�����B

	while( ul_Wait < ul_time );			// �w�莞�Ԍo�߂���܂ŃE�F�C�g
	
	return;
}


// *************************************************************************
//   �@�\		�F �w�肵���t���[�J�E���g�ԁAS/W�E�F�C�g����B
//   ����		�F �Ȃ�
//   ����		�F ul_cnt: �J�E���g�l
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.28			sato			�V�K
// *************************************************************************/
PUBLIC void TIME_waitFree( ULONG ul_cnt )
{
	while( ul_cnt-- );			// 0�ɂȂ�܂Ńf�B�N�������g
}

// *************************************************************************
//   �@�\		�F ���[�h�����Z�ύX����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.07.02			sato			�V�K
// *************************************************************************/
PRIVATE void MODE_inc( void )
{
	en_Mode++;		// ���[�h�����Z
	
	/* �ő�l�`�F�b�N */
	if( MODE_MAX == en_Mode ){
		en_Mode = MODE_0;
	}
	
	/* ���[�h�\�� */
	switch( en_Mode ){
	
		case MODE_0:
			LED4 = 0x00;
			break;

		case MODE_1:
			LED4 = 0x01;
			break;

		case MODE_2:
			LED4 = 0x02;
			break;

		case MODE_3:
			LED4 = 0x03;
			break;

		case MODE_4:
			LED4 = 0x04;
			break;

		case MODE_5:
			LED4 = 0x05;
			break;

		case MODE_6:
			LED4 = 0x06;
			break;

		case MODE_7:
			LED4 = 0x07;
			break;

		case MODE_8:
			LED4 = 0x08;
			break;
			
		case MODE_9:
			LED4 = 0x09;
			break;
			
		case MODE_10:
			LED4 = 0x0a;
			break;
			
		case MODE_11:
			LED4 = 0x0b;
			break;
			
		case MODE_12:
			LED4 = 0x0c;
			break;
			
		case MODE_13:
			LED4 = 0x0d;
			break;
			
		case MODE_14:
			LED4 = 0x0e;
			break;
			
		case MODE_15:
			LED4 = 0x0f;
			break;

		default:
			break;
	}
}

// *************************************************************************
//   �@�\		�F ���[�h0�����s����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.08.28			sato			�V�K
// *************************************************************************/
PRIVATE void MODE_exe0( void )
{
	enMAP_HEAD_DIR		en_endDir2;	//MAPcmd�`�F�b�N�p
	
	/* ���[�h�\�� */
	switch( en_Mode ){
	
		case MODE_0:
			LED4 = LED4_ALL_ON;
			while(1){
				
				printf("[�d���d��] %5.2f [mV] \n\r",BAT_getLv() );
				TIME_wait(500);
			}
			break;

		case MODE_1:
			LED4 = LED4_ALL_ON;
			CTRL_clrData();
			while(1){
				printf("   [�W���C���p�x]%5.2f [SPI�W���C��]%x \r", 
					GYRO_getNowAngle(),recv_spi_gyro()
				);
				TIME_wait( 500 );
			}
			break;
		
		case MODE_2:
			LED4 = LED4_ALL_ON;
			while(1){

				printf("   �����Z���T [R_F]%5d [L_F]%5d [R_S]%5d [L_S]%5d \r", 
					(int)DIST_getNowVal(DIST_SEN_R_FRONT),
					(int)DIST_getNowVal(DIST_SEN_L_FRONT),
					(int)DIST_getNowVal(DIST_SEN_R_SIDE),
					(int)DIST_getNowVal(DIST_SEN_L_SIDE)
				);
				TIME_wait( 300 );
			}
			break;
		
		case MODE_3:
			LED4 = LED4_ALL_ON;
			while(1){
				printf("[who am i]%x\n\r",recv_spi_who());
				TIME_wait(500);
			}
			break;

		case MODE_4:
			LED4 = LED4_ALL_ON;
			while(1){
				printf("�G���R�[�_ [R]=%d [L]=%d \r",ENC_R_TCNT,ENC_L_TCNT);
				TIME_wait(50);
			}
			break;

		case MODE_5:
			LED4 = LED4_ALL_ON;
			MAP_showLog();
			break;

		case MODE_6:
			LED4 = LED4_ALL_ON;
			MAP_setPos( 0, 0, NORTH );								// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir2 );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();
			MAP_showCmdLog();
			break;

		case MODE_7:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_8:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_9:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_10:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_11:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_12:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_13:
			LED4 = LED4_ALL_ON;
			break;

		case MODE_14:
			LED4 = LED4_ALL_ON;
			turntable();
			break;

		case MODE_15://���[�h15�͗��E�p�̂��߃v���O�����͐ݒ肵�Ȃ�
			LED4 = LED4_ALL_ON;
			break;
			
		default:
			break;
	}
}

// *************************************************************************
//   �@�\		�F ���[�h�����s����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.13			�O��			�V�K
// *************************************************************************/
PRIVATE void MODE_exe( void )
{
//	USHORT *read;
	enMAP_HEAD_DIR		en_endDir;
	GYRO_SetRef();
//	log_flag_on();	//���O�֐����s�p�t���O�@���ɂ͍폜
	/* ���[�h�\�� */
	switch( en_Mode ){
	
		case MODE_0:	//���W���[�������p�v���O�����Q
			LED4 = LED4_ALL_ON;
			en_Mode = MODE_0;	//���ӁFMODE_inc�𗘗p���邽�ߍŏ���en_Mode���������@�Ō��en_Mode��߂����삪�K�v
			TIME_wait(100);
			LED4 = LED4_ALL_OFF;
			while(1){
				if ( SW_ON == SW_INC_PIN ){
					MODE_inc();								// ���[�h��1�i�߂�
					TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
					printf("mode selecting_0\r\n");
				}
				else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
					MODE_exe0();								// ���[�h���s
					TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
					if (en_Mode == MODE_15)break;
				}

			}
			en_Mode = MODE_0;
			break;

		case MODE_1:
			LED4 = LED4_ALL_ON;
		
			break;
			
		case MODE_2:
			LED4 = LED4_ALL_ON;
			
			break;

		case MODE_3:	//���x700�ł̃X�����[���`�F�N�v���O����
			LED4 = LED4_ALL_ON;
//			TIME_wait(200);
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)500 );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_VERY_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_VERY_FAST );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;

			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 5000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000


//			PARAM_makeSra( (FLOAT)600, 150.0f, 5000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
//			PARAM_makeSra( (FLOAT)600, 150.0f, 6000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
//			PARAM_makeSra( (FLOAT)600, 200.0f, 7000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
//			PARAM_makeSra( (FLOAT)600, 300.0f, 8000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
			
//			PARAM_makeSra( (FLOAT)700, 250.0f, 6000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
//			PARAM_makeSra( (FLOAT)700, 250.0f, 7000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
//			PARAM_makeSra( (FLOAT)700, 450.0f, 8000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
//			PARAM_makeSra( (FLOAT)700, 500.0f, 9500.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
/*
			Dist_autocalibration();
			Failsafe_flag_off();
			
			// ���H�T�� 
//			POS_clr();			// debug
//			POS_sta();			// debug
			MAP_setPos( 0, 0, NORTH );							// �X�^�[�g�ʒu
			MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA );			// �S�[���ݒ�
//			POS_stop();			// debug
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
			}
			// �A��̃X�����[���T�� 
			TIME_wait(1000);
			LED4 = LED4_ALL_OFF;
			MAP_searchGoal( 0, 0, SEARCH, SEARCH_SURA );
			TIME_wait(1000);
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
//			PARAM_setCntType( TRUE );								// �ŒZ���s
			MAP_setPos( 0, 0, NORTH );								// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();
			}
*/			
//			Dist_autocalibration();
			
			log_flag_on();

			MOT_goBlock_FinSpeed( 1.0, 500 );
			MOT_goSla( MOT_R45S_S2N, PARAM_getSra( SLA_45 ) );
			MOT_goBlock_FinSpeed( 2.0, 0 );

//			MOT_goSkewBlock_FinSpeed(0.5,700);
//			MOT_goSla( MOT_L90S_N, PARAM_getSra( SLA_N90 ) );
//			MOT_goSkewBlock_FinSpeed(0.5,0);

			log_flag_off();	
			break;

		case MODE_4:
			LED4 = LED4_ALL_ON;
//			TIME_wait(200);
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
//			Dist_autocalibration();
			Failsafe_flag_off();
			
			/* ���H�T�� */
//			POS_clr();			// debug
//			POS_sta();			// debug
			MAP_setPos( 0, 0, NORTH );							// �X�^�[�g�ʒu

			log_flag_on();

			MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA );			// �S�[���ݒ�

			log_flag_off();

//			POS_stop();			// debug
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
			}
			/* �A��̃X�����[���T�� */
			TIME_wait(1000);
			LED4 = LED4_ALL_OFF;

//			log_flag_on();

			MAP_searchGoal( 0, 0, SEARCH, SEARCH_SURA );

//			log_flag_off();

			TIME_wait(1000);
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
//			PARAM_setCntType( TRUE );								// �ŒZ���s
			MAP_setPos( 0, 0, NORTH );								// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();
			}
			break;

		case MODE_5:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*4);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_NORMAL );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			Failsafe_flag_off();
//			calibration();

			log_read2();
			break;

		case MODE_6:
			LED4 = LED4_ALL_ON;
			
			TIME_wait(1000);
//			MOT_setTrgtSpeed(SEARCH_SPEED*6);
//			MOT_setSuraStaSpeed( SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
//			PARAM_setSpeedType( PARAM_ST,   PARAM_FAST );							// [���i] ���x����
//			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
//			PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			
//			PARAM_makeSra( RUN_SPEED, 450.0f, 5000.0f, SLA_90 );
			
			LED4 = LED4_ALL_OFF;
			MAP_Goal_change_x();
			TIME_wait(500);
			MAP_Goal_change_y();

			break;

		case MODE_7:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);

			MOT_setTrgtSpeed(SEARCH_SPEED*6);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_NORMAL );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			Failsafe_flag_off();
			
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			MAP_drive( MAP_DRIVE_SKEW );
			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();

/*
			//�V�X�e������p
			DCM_setDirCw(DCM_R);	//���v���
			DCM_setDirCcw(DCM_L);	//�����v���
			DCM_setPwmDuty(DCM_R,200);	//PWM�ݒ�ADuty20%
			DCM_setPwmDuty(DCM_L,200);	//PWM�ݒ�ADuty20%
			DCM_staMotAll();
			TIME_wait(1000);
			DCM_stopMot( DCM_R );				// ���[�^��~
			DCM_stopMot( DCM_L );				// ���[�^��~
*/
			break;

		case MODE_8:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*8);
			MOT_setSuraStaSpeed( SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			
//			PARAM_makeSra( (FLOAT)700, 250.0f, 6000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
//			PARAM_makeSra( (FLOAT)700, 250.0f, 7000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
//			PARAM_makeSra( (FLOAT)700, 450.0f, 8500.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
//			PARAM_makeSra( (FLOAT)700, 500.0f, 9500.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
			
			
			Failsafe_flag_off();
//			PARAM_makeSra( (FLOAT)SEARCH_SPEED*1.5, 400.0f, 7500.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			
//			log_flag_on();

			MAP_drive( MAP_DRIVE_SURA );
			
//			log_flag_off();

			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();
			
//			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 2500.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
//			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
//			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
//			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

			break;
			
		case MODE_9:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*10);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_VERY_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			Failsafe_flag_off();
//			map_copy();
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			MAP_drive( MAP_DRIVE_SURA );
			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();
//			MAP_showCmdLog();
			break;
			
		case MODE_10:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*8);
			MOT_setSuraStaSpeed( (FLOAT)600 );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_FAST );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;

			PARAM_makeSra( (FLOAT)600, 150.0f, 5000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)600, 150.0f, 6000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)600, 200.0f, 7000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)600, 300.0f, 8000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
			
			Failsafe_flag_off();
			
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			MAP_drive( MAP_DRIVE_SURA );
			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();

			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 2500.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

			break;
			
		case MODE_11:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*10);
			MOT_setSuraStaSpeed( (FLOAT)600 );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_VERY_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_FAST );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;

			PARAM_makeSra( (FLOAT)600, 150.0f, 5000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)600, 150.0f, 6000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)600, 200.0f, 7000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)600, 300.0f, 8000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
			
			Failsafe_flag_off();
			
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			MAP_drive( MAP_DRIVE_SURA );
			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();

			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 2500.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

			break;
			
		case MODE_12:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED*12);
			MOT_setSuraStaSpeed( (FLOAT)700);							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_VERY_FAST );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_FAST );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			
			PARAM_makeSra( (FLOAT)700, 250.0f, 6000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)700, 250.0f, 7000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)700, 450.0f, 8500.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)700, 500.0f, 9500.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000
			
			Failsafe_flag_off();
			
			MAP_setPos( 0, 0, NORTH );												// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();													// �΂߃R�}���h�쐬
			MAP_drive( MAP_DRIVE_SURA );
			TIME_wait(500);
			MOT_turn(MOT_R180);
			MAP_actGoalLED();

			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 2500.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

			break;
			
		case MODE_13:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [���i] ���x����
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [����] ���x����
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [�X��] ���x����
			LED4 = LED4_ALL_OFF;
			
			/* ���H�T�� */
//			POS_clr();			// debug
//			POS_sta();			// debug
			MAP_setPos( 0, 0, NORTH );							// �X�^�[�g�ʒu
			MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_TURN );			// �S�[���ݒ�
//			POS_stop();			// debug
			
			/* �A��̃X�����[���T�� */
			TIME_wait(1000);
			LED4 = LED4_ALL_OFF;
			MAP_searchGoal( 0, 0, SEARCH, SEARCH_TURN );
			TIME_wait(1000);
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
//			PARAM_setCntType( TRUE );								// �ŒZ���s
			MAP_setPos( 0, 0, NORTH );								// �X�^�[�g�ʒu
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
			MAP_makeSkewCmdList();
			}
			break;
			
		case MODE_14:
			LED4 = LED4_ALL_ON;
			map_erase();
			LED4 = LED4_ALL_OFF;
			
			break;
			
		case MODE_15:
			LED4 = LED4_ALL_ON;
			TIME_wait(1000);
			map_copy();
			LED4 = LED4_ALL_OFF;
			
			break;
			
		default:
			break;
	}
}

// *************************************************************************
//   �@�\		�F main����
//   ����		�F �������I�������Ȃ��悤�ɂ���B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.21			sato		�V�K
//		v1.1		2018.06.25			sato		�^�C�}�[���荞�݂̏�����
// *************************************************************************/
void main(void)
{	
	/* ������ */
	CPU_init();						// [CPU] ���W�X�^/GPIO/AD/TIMER�Ȃ�
	TIME_init();						// [SYS] �^�C�}�[
	HAL_init();						// [HAL] �n�[�h�E�G�A�h���C�o
	DIST_init();
	MAP_Goal_init();
	RX631_staTimer();					// [CPU] �^�C�}�X�^�[�g
//	printf("set1\n\r");
	SYS_start();						// [SYS] ����J�n
	hw_dflash_init();
	MAP_init();
	
	recv_spi_init();					//SPI�C�j�V�����C�Y
	TIME_wait(100);
	GYRO_SetRef();						// [GYRO] �W���C���̊�l�̎擾
//	TIME_wait(20);

	/* big roop */
	while(1){
		if ( SW_ON == SW_INC_PIN ){
			MODE_inc();								// ���[�h��1�i�߂�
			TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
		printf("mode selecting\r\n");
		}
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
			MODE_exe();								// ���[�h���s
			TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
		}

	}
}

// *************************************************************************
//   �@�\		�F ���荞�݊֐��A�V�X�e���p(msec�^�C�})
//   ����		�F �Ȃ�
//   ����		�F MTU0��TGRA���荞�݁A1msec���Ɋ֐������s�����B
//				   ����Ƀ}�C�R�������삵�Ă���ꍇ�A1msec������LED���_�ł���
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.23			sato		�V�K
// *************************************************************************/
PUBLIC void INTC_sys(void)
{
//	static USHORT i = 0;
	
	/*�V�X�e������m�FLED*/
/*	if( i==500 ){
//		LED0 = ~LED0;	//L�`�J
		i=0;
	}
	else{
		i++;
	}
*/	
	/* ---------- */
	/*  �������v  */
	/* ---------- */
	uc_Msec++;					// msec
	if( uc_Msec > 999 ){		// msec �� sec
		uc_Msec  = 0;
		uc_Sec++;
	}
	if( uc_Sec > 59 ){			// sec �� min
		uc_Sec = 0;
		uc_Min++;
	}
	
	/* ----------------------- */
	/*  S/W�E�F�C�g�E�J�E���^  */
	/* ----------------------- */
	ul_Wait++;
	ul_Wait %= 6000000;			// 10 min (= 6000000 �J�E���g) �� 0 �N���A
	
	CTRL_pol();
}

// *************************************************************************
//   �@�\		�F ������J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.25			�O��			�V�K
// *************************************************************************/
PRIVATE void SYS_start( void )
{
//	UCHAR	i;

	/* LED��ʁ̉���4bit�_�� */
/*	for( i = 0; i < 2; i++ ){
	
		LED4 = 0x03;
		TIME_wait(50);
		LED4 = 0x0C;
		TIME_wait(50);
	}*/
//	LED4 = LED4_ALL_ON;		// �S����

	/* �^�C�g���\�� */
	printf(" ����������������������������������������������\r\n");
	printf(" �� Robo Name  : �T���V���C��(�E�ցE)/~~     ��\r\n");
	printf(" �� Developer  : T.Togawa & Oki              ��\r\n");
	printf(" �� Version    : Protype1���@                ��\r\n");
	printf(" �� Target     : ����p                      ��\r\n");
	printf(" �� Project By : Hosei Univ. Denken Group    ��\r\n");
	printf(" ����������������������������������������������\r\n");

	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 5000.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 4000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

//	printf("600\n\r");
//	PARAM_makeSra( (FLOAT)600, 350.0f, 4500.0f, SLA_45 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v200 2000	T	200 2000
//	PARAM_makeSra( (FLOAT)600, 350.0f, 5000.0f, SLA_90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 3500		200 4000
//	PARAM_makeSra( (FLOAT)600, 400.0f, 6500.0f, SLA_135 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v300 4500		300 4000
//	PARAM_makeSra( (FLOAT)600, 550.0f, 7500.0f, SLA_N90 );		// �i�����x[mm/s]�A�p�����x[rad/s^2]�A��G[mm/s^2]�A�X�����[���^�C�v500 5000		500 5000

	
}

// *************************************************************************
//   �@�\		�F ���荞�݊֐��A�Z���T�p
//   ����		�F �Ȃ�
//   ����		�F MTU4��TGRA���荞�݁A0.25msec���Ɋ֐������s�����B
//				�F ����Ƀ}�C�R�������삵�Ă���ꍇ�A250usec������
//				�F �����Z���T0 �� �����Z���T1 �� �����Z���T2 �� �W���C���Z���T �̏��ŃX�L��������B
//				�F �]���āA1�Z���T�̃X�L����������1msec�ƂȂ�B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.11.04			�O��			�V�K
// *************************************************************************/
PUBLIC void INTC_sen( void )
{
	static UCHAR	i = 0;
	
	/* �Z���T����  */
	switch( i ){
		case 0:		// �W���C���Z���T
			GYRO_Pol();
			break;
		
		case 1:		// �O�ǃZ���T
			DIST_Pol_Front();
			break;
		
		case 2:		// ���ǃZ���T
			DIST_Pol_Side();
			break;
		
		case 3:		// �΂ߕǃZ���T
			// �΂߃Z���T�͎g�p���Ȃ�
			break;
		
	}
	
	i = ( i + 1 ) % 4;			// roop
	
	return;
}


// *************************************************************************
//   �@�\		�F ���荞�݊֐��A�f�[�^�t���b�V���p
//   ����		�F �Ȃ�
//   ����		�F MTU2��TGRA���荞�݁A5msec���Ɋ֐������s�����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.29			sato			�V�K
// *************************************************************************/
PUBLIC void INTC_flash( void )
{
//	static UCHAR	i = 0;
	
//	log_write_base(i);
	
//	i = ( i + 1 ) % 4;			// roop
	
//	return;
}

//�g��Ȃ��R�[�h�i�����j
#ifdef __cplusplus
void abort(void)
{

}
#endif