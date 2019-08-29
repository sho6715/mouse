// *************************************************************************
//   ���{�b�g��		�F AEGIS
//   �T�v		�F hal_dist�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.15			sato			�V�K�i�t�@�C���̃C���N���[�h�j
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
#define DIST_NO_WALL_DIV_FILTER				( 200 )						// �ǂȂ��Ƃ��鍷���l
#define DIST_REF_UP					( 50 )						// �ǂȂ��Ɣ��f����ۂɊ�l�ɉ��Z����l
/*
#define R_FRONT_REF					( 350 )					// �E�O�ǁA��l�@���̒����ɂ����ĕǂ������l275
#define L_FRONT_REF					( 280 )					// ���O�ǁA��l210
#define R_SIDE_REF					( 280 )						// �E���ǁA��l230
#define L_SIDE_REF					( 240 )						// �����ǁA��l180
#define R_FRONT_WALL					( 70 )						// �E�O�ǁA�ǌ��m�l�@���̒������炸�炵�Č����Ƃ��̒l�i�ǂ����邽�߂̒l�j45
#define L_FRONT_WALL					( 50 )					// ���O�ǁA�ǌ��m�l40
#define R_SIDE_WALL					( 115 )						// �E���ǁA�ǌ��m�l100
#define L_SIDE_WALL					( 70 )					// �����ǁA�ǌ��m�l80
*/
#define R_FRONT_REF					( 750 )					// �E�O�ǁA��l�@���̒����ɂ����ĕǂ������l250
#define L_FRONT_REF					( 620 )					// ���O�ǁA��l210
#define R_SIDE_REF					( 550 )						// �E���ǁA��l290
#define L_SIDE_REF					( 520 )						// �����ǁA��l250
#define R_FRONT_WALL					( 230 )						// �E�O�ǁA�ǌ��m�l�@���̒������炸�炵�Č����Ƃ��̒l�i�ǂ����邽�߂̒l�j55
#define L_FRONT_WALL					( 230 )					// ���O�ǁA�ǌ��m�l55
#define R_SIDE_WALL					( 450 )						// �E���ǁA�ǌ��m�l140
#define L_SIDE_WALL					( 430 )					// �����ǁA�ǌ��m�l100
#define R_FRONT_SKEW_ERR1				( 80 )//��
#define L_FRONT_SKEW_ERR1				( 70 )
#define R_FRONT_SKEW_ERR2				( 192 )
#define L_FRONT_SKEW_ERR2				( 160 )
#define R_FRONT_SKEW_ERR3				( 250 )
#define L_FRONT_SKEW_ERR3				( 250 )
#define R_FRONT_CTRL					( 800 )
#define L_FRONT_CTRL					( 680 )
#define R_FRONT_NOCTRL					( 850 )
#define L_FRONT_NOCTRL					( 770 )


#define		SEN_WAIT_CNT		(175)		//�Z���T�̔�������҂�(�����l)
#define		DIST_REF_NUM		(400)

/* �����Z���T(���ω�) */
#define R_FRONT_WALL_GAIN			( 0.13f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define L_FRONT_WALL_GAIN			( 0.16f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define R_SIDE_WALL_GAIN			( 0.43f )							// �E���ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define L_SIDE_WALL_GAIN			( 0.60f )							// �����ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define R_FRONT_WALL_CTRL_GAIN		( 0.74f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂷��l
#define L_FRONT_WALL_CTRL_GAIN		( 0.82f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂷��l
#define R_FRONT_WALL_NO_CTRL_GAIN	( 2.13f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂵�Ȃ��l
#define L_FRONT_WALL_NO_CTRL_GAIN	( 1.95f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂵�Ȃ��l
//#define R_FRONT_WALL_HIT_GAIN		( 2.73f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
//#define L_FRONT_WALL_HIT_GAIN		( 3.41f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
#define R_FRONT_SKEW_ERR1_GAIN		( 0.29f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l1
#define R_FRONT_SKEW_ERR2_GAIN		( 0.7f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l2
#define R_FRONT_SKEW_ERR3_GAIN		( 1.23f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l3
#define L_FRONT_SKEW_ERR1_GAIN		( 0.28f )							// ���O�ǁA�΂ߑ��s���̕␳臒l1
#define L_FRONT_SKEW_ERR2_GAIN		( 0.7f )							// ���O�ǁA�΂ߑ��s���̕␳臒l2
#define L_FRONT_SKEW_ERR3_GAIN		( 1.23f )							// ���O�ǁA�΂ߑ��s���̕␳臒l3
#define R_FRONT_CTRL_GAIN		( 1.5f )
#define L_FRONT_CTRL_GAIN		( 1.5f ) 
#define R_FRONT_NOCTRL_GAIN		( 0.5f )
#define L_FRONT_NOCTRL_GAIN		( 0.5f )

//**************************************************
// �񋓑́ienum�j
//**************************************************


//**************************************************
// �\���́istruct�j
//**************************************************
/* �����Z���T���i�O�ǂ̂݁A�f�[�^�t���b�V���p�\���̂Ƃ��Ă��g�p����j */
typedef struct{
	SHORT		s_wallHit;					///< @var : �ǂɓ������Ă��Ă����������Ȃ��l         ( AD �l ) �i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j					///< @var : �΂ߑ��s���̕␳臒l3                    ( AD �l )
	SHORT		s_skewErr1;					///< @var : �΂ߑ��s���̕␳臒l1                    ( AD �l )
	SHORT		s_skewErr2;					///< @var : �΂ߑ��s���̕␳臒l2                    ( AD �l )
	SHORT		s_skewErr3;					///< @var : �΂ߑ��s���̕␳臒l3
}stDIST_FRONT_SEN;


/* �����Z���T���i�S�Z���T���ʁA�f�[�^�t���b�V���p�\���݂̂̂Ɏg�p�j */
typedef struct{
	SHORT		s_ref;						///< @var : ���̒��S�ɒu�������̋����Z���T�̊�l ( AD �l )
	SHORT		s_limit;					///< @var : �����Z���T��臒l                         ( AD �l ) ( ���̒l���傫���ꍇ�A�ǂ���Ɣ��f���� )
	SHORT		s_ctrl;						///< @var : ����L��������ۂ�臒l                   ( AD �l ) ��ɑO�ǂŎg�p
	SHORT		s_noCtrl;					///< @var : �ǂɋ߂����邽�ߐ��䖳��������ۂ�臒l   ( AD �l ) ��ɑO�ǂŎg�p
}stDIST_SEN_DATA;

/* �����Z���T���i�S�Z���T���ʁj ���@hal.h�Ɉڍs*/
typedef struct{
	SHORT		s_now;						// LED �_�����̋����Z���T�̌��ݒl           ( AD �l )
	SHORT		s_old;						// LED �_�����̋����Z���T��1�O�̒l        ( AD �l )
	SHORT		s_limit;					// �����Z���T��臒l                         ( AD �l ) ( ���̒l���傫���ꍇ�A�ǂ���Ɣ��f���� )
	SHORT		s_ref;						// ���̒��S�ɒu�������̋����Z���T�̊�l ( AD �l )
	SHORT		s_offset;					// LED �������̋����Z���T�̒l               ( AD �l )
	SHORT		s_ctrl;						// ����L��������ۂ�臒l                   ( AD �l ) ��ɑO�ǂŎg�p
	SHORT		s_noCtrl;					// �ǂɋ߂����邽�ߐ��䖳��������ۂ�臒l   ( AD �l ) ��ɑO�ǂŎg�p
}stDIST_SEN;

//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �����Z���T */
PRIVATE stDIST_SEN		st_sen[DIST_SEN_NUM];					// �����Z���T
PRIVATE stDIST_FRONT_SEN	st_senF[DIST_SEN_L_FRONT+1];	// �����Z���T(�O�ǂ̂�)
PRIVATE LONG  l_Lside_DistRef; 									// 
PRIVATE LONG  l_Rside_DistRef; 
PRIVATE LONG  l_Lfront_DistRef;
PRIVATE LONG  l_Rfront_DistRef; 

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************


// *************************************************************************
//   �@�\		�F HAL_dist������������B
//   ����		�F �Ȃ�
//   ����		�F �����ϐ��Ȃǂ��N���A����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑�     �F�N�����ɓ���
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.15			sato			�V�K
// *************************************************************************/
PUBLIC void DIST_init( void )
{
	/* �����Z���T */
	l_Lside_DistRef = 0; 									// �W���C���Z���T�̊�l
	l_Rside_DistRef = 0; 
	l_Lfront_DistRef = 0;
	l_Rfront_DistRef = 0;

	memset( st_sen, 0, sizeof(st_sen) );				// �����Z���T(�S�Z���T����)
	memset( st_senF, 0, sizeof(st_senF) );				// �����Z���T(�O�Z���T�̂�)
	st_sen[DIST_SEN_R_FRONT].s_ref       = R_FRONT_REF;
	st_sen[DIST_SEN_L_FRONT].s_ref       = L_FRONT_REF;
	st_sen[DIST_SEN_R_SIDE].s_ref        = R_SIDE_REF;
	st_sen[DIST_SEN_L_SIDE].s_ref        = L_SIDE_REF;
	st_sen[DIST_SEN_R_FRONT].s_limit     = R_FRONT_WALL;
	st_sen[DIST_SEN_L_FRONT].s_limit     = L_FRONT_WALL;
	st_sen[DIST_SEN_R_SIDE].s_limit      = R_SIDE_WALL;
	st_sen[DIST_SEN_L_SIDE].s_limit      = L_SIDE_WALL;
	st_senF[DIST_SEN_R_FRONT].s_skewErr1	= R_FRONT_SKEW_ERR1;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1	= L_FRONT_SKEW_ERR1;
	st_senF[DIST_SEN_R_FRONT].s_skewErr2	= R_FRONT_SKEW_ERR2;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2	= L_FRONT_SKEW_ERR2;
	st_senF[DIST_SEN_R_FRONT].s_skewErr3	= R_FRONT_SKEW_ERR3;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3	= L_FRONT_SKEW_ERR3;
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = R_FRONT_NOCTRL;//�����l
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = L_FRONT_NOCTRL;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = R_FRONT_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = L_FRONT_CTRL;

}

// *************************************************************************
//   �@�\		�F �����Z���T�̒l���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id )
{
	return st_sen[en_id].s_now;
}


// *************************************************************************
//   �@�\		�F ��l�ƌ��ݒl�̕΍����擾����
//   ����		�F �Ȃ�
//   ����		�F �ǂ̐؂�ڕ␳�Ή����{
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �΍�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void DIST_getErr( LONG* p_err )
{
	VSHORT	s_threshold_R = 0;		// �E�Z���T��臒l
	VSHORT	s_threshold_L = 0;		// ���Z���T��臒l
	SHORT	s_temp;
/*	
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = 340;//�����l
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = 260;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = 170;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = 130;
*/	
	/* ---------- */
	/*  �E�ǐ���  */
	/* ---------- */
	/* �ǂ̐؂�ڑ΍� */
	// �}���ɃZ���T�̒l���ω������ꍇ�́A�ǂ̗L���̊�l��臒l�ɕύX����
	s_temp = st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_ref + DIST_REF_UP;		// ��l�{����ǂ̑��݂���臒l�ɂ���
	}
	else{
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_limit;		// �ʏ�ʂ�
	}

	/* ---------- */
	/*  ���ǐ���  */
	/* ---------- */
	/* �ǂ̐؂�ڑ΍� */
	// �}���ɃZ���T�̒l���ω������ꍇ�́A�ǂ̗L���̊�l��臒l�ɕύX����
	s_temp = st_sen[DIST_SEN_L_SIDE].s_now - st_sen[DIST_SEN_L_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_ref + DIST_REF_UP;		// ��l�{����ǂ̑��݂���臒l�ɂ���
	}
	else{
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_limit;		// �ʏ�ʂ�
	}

	/* ------------ */
	/*  ����l�Z�o  */
	/* ------------ */
	*p_err = 0;		// �N���A
	
	/* �O�ǂ����̂������߂��� */
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_noCtrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_noCtrl )
	){
//		printf("[Val]%6d �O�ǂ����̂������߂� 	\n\r", *p_err);
		*p_err = 0;
	}
	/* �O�� */
	else if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_ctrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_ctrl )
	){
		*p_err = ( st_sen[DIST_SEN_L_FRONT].s_now - st_sen[DIST_SEN_L_FRONT].s_ref ) -
				 ( st_sen[DIST_SEN_R_FRONT].s_now - st_sen[DIST_SEN_R_FRONT].s_ref );
//		printf("[Val]%6d �O�ǐ��� 	\n\r", *p_err);
	}
	/* �E�ǂƍ��ǂ��� */
	else if( ( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ) && ( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now )
	){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) + 
				 ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now );
//		printf("[Val]%6d ���ǐ��� 	\n\r", *p_err);
//		LED4 = 0x09;
	}
	/* �E�ǂ��� */
	else if( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) * 2;
//		printf("[Val]%6d �E�ǐ��� 	\n\r", *p_err);
//		LED4 = 0x08;
	}
	/* ���ǂ��� */
	else if( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now ) * 2;
//		printf("[Val]%6d ���ǐ��� 	\n\r", *p_err);
//		LED4 = 0x01;
	}
}

// *************************************************************************
//   �@�\		�F ��l����̕΍���Ԃ��擾����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.15			����			�V�K
// *************************************************************************/
PUBLIC void DIST_getErrSkew( LONG* p_err )
{
	*p_err =0;
	
	/* �i�s�����ɕǂ����݂���ꍇ�ɂ悯�铮����s�� */
	if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr3 ){
		*p_err = 150;
//		LED_on(LED0);
//		LED_off(LED1);
		
//		printf("�E�O�����߂�  [NOW]%d > [ERR3]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr3 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr3 ){
		*p_err = -150;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("���O�����߂�  [NOW]%d > [ERR3]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr3 );
	}
	else if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr2 ){
		*p_err = 100;
//		LED_on(LED0);
//		LED_off(LED1);
//		printf("�E�O�������߂�  [NOW]%d > [ERR2]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr2 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr2 ){
		*p_err = -100;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("���O�������߂�  [NOW]%d > [ERR2]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr2 );
	}
	else if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr1 ){
		*p_err = 50;
//		LED_on(LED0);
//		LED_off(LED1);
//		printf("�E�O���߂�  [NOW]%d > [ERR1]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr1 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr1 ){
		*p_err = -50;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("���O���߂�  [NOW]%d > [ERR1]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr1 );
	}
	else{
//		LED_off(LED0);
//		LED_off(LED1);
	}
	
}

// *************************************************************************
//   �@�\		�F �����Z���T�p�i�O�ǁj�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC void DIST_Pol_Front( void )
{
	/* ���������̒l�擾 */
	S12AD.ADANS0.WORD 		= 0x0012;		// AN1/4 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_FRONT].s_offset = (SHORT)S12AD.ADDR4;
	st_sen[DIST_SEN_L_FRONT].s_offset = (SHORT)S12AD.ADDR1;

	/* �O��LED�_�� */
	LED_DIST_RF = ON;
	LED_DIST_LF = ON;
	
	/* ��������҂� */
	TIME_waitFree( SEN_WAIT_CNT );

	/* �������̒l�Ɩ��������̒l�ō������擾 */
	S12AD.ADANS0.WORD 		= 0x0012;		// AN1/2 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_FRONT].s_old = st_sen[DIST_SEN_R_FRONT].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_L_FRONT].s_old = st_sen[DIST_SEN_L_FRONT].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_R_FRONT].s_now = (SHORT)S12AD.ADDR4 - st_sen[DIST_SEN_R_FRONT].s_offset;		// ���ݒl��������
	st_sen[DIST_SEN_L_FRONT].s_now = (SHORT)S12AD.ADDR1 - st_sen[DIST_SEN_L_FRONT].s_offset;		// ���ݒl��������
	
	/* �O��LED���� */
	LED_DIST_RF = OFF;
	LED_DIST_LF = OFF;
}


// *************************************************************************
//   �@�\		�F �����Z���T�p�i���ǁj�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC void DIST_Pol_Side( void )
{
	/* ���������̒l�擾 */
	S12AD.ADANS0.WORD 		= 0x000C;		// AN2/3 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_SIDE].s_offset = (SHORT)S12AD.ADDR3;
	st_sen[DIST_SEN_L_SIDE].s_offset = (SHORT)S12AD.ADDR2;

	/* ����LED�_�� */
	LED_DIST_RS = ON;
	LED_DIST_LS = ON;
	
	/* ��������҂� */
	TIME_waitFree( SEN_WAIT_CNT );

	/* �������̒l�Ɩ��������̒l�ō������擾 */
	S12AD.ADANS0.WORD 		= 0x000C;		// AN0/3 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD�ϊ��J�n
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_SIDE].s_old = st_sen[DIST_SEN_R_SIDE].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_L_SIDE].s_old = st_sen[DIST_SEN_L_SIDE].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_R_SIDE].s_now = (SHORT)S12AD.ADDR3 - st_sen[DIST_SEN_R_SIDE].s_offset;		// ���ݒl��������
	st_sen[DIST_SEN_L_SIDE].s_now = (SHORT)S12AD.ADDR2 - st_sen[DIST_SEN_L_SIDE].s_offset;		// ���ݒl��������
	
	/* ����LED���� */
	LED_DIST_RS = OFF;
	LED_DIST_LS = OFF;
}

// *************************************************************************
//   �@�\		�F �O�ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_FRONT( void )
{
	BOOL bl_res 		= FALSE;
//	printf("DIST_SEN_R_FRONT %5d \r\n",st_sen[DIST_SEN_R_FRONT].s_limit);
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_limit ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_limit )
	){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F �E�ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_R_SIDE( void )
{
	BOOL bl_res 		= FALSE;
	
	if( st_sen[DIST_SEN_R_SIDE].s_now > st_sen[DIST_SEN_R_SIDE].s_limit ){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F ���ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_L_SIDE( void )
{
	BOOL bl_res 		= FALSE;
	
	if( st_sen[DIST_SEN_L_SIDE].s_now > st_sen[DIST_SEN_L_SIDE].s_limit ){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F ���ǃI�[�g�L�����u���[�V����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.28			sato			�V�K
// *************************************************************************/
PRIVATE void Dist_side_auto_calibration (void)
{
	USHORT i;
	ULONG ul_Lref = 0;
	ULONG ul_Rref = 0;
	
	/* �f�[�^�T���v�����O */
	for( i=0; i<DIST_REF_NUM; i++){			// 100��T���v�����O�������ϒl����̒l�Ƃ���B
		ul_Lref += st_sen[DIST_SEN_L_SIDE].s_now;
		ul_Rref += st_sen[DIST_SEN_R_SIDE].s_now;
		TIME_wait(1);
	}
	
	/* ��l�Z�o�i���ϒl�j */
	l_Lside_DistRef = ul_Lref / DIST_REF_NUM ;		// ���x��100�{�ɂ���
	l_Rside_DistRef = ul_Rref / DIST_REF_NUM ;
	
	st_sen[DIST_SEN_R_SIDE].s_ref        = l_Rside_DistRef;
	st_sen[DIST_SEN_L_SIDE].s_ref        = l_Lside_DistRef;
	st_sen[DIST_SEN_R_SIDE].s_limit      = l_Rside_DistRef * R_SIDE_WALL_GAIN;
	st_sen[DIST_SEN_L_SIDE].s_limit      = l_Lside_DistRef * L_SIDE_WALL_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr1	= l_Rside_DistRef * R_FRONT_SKEW_ERR1_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1	= l_Lside_DistRef * L_FRONT_SKEW_ERR1_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr2	= l_Rside_DistRef * R_FRONT_SKEW_ERR2_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2	= l_Lside_DistRef * L_FRONT_SKEW_ERR2_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr3	= l_Rside_DistRef * R_FRONT_SKEW_ERR3_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3	= l_Lside_DistRef * L_FRONT_SKEW_ERR3_GAIN;
}

// *************************************************************************
//   �@�\		�F �O�ǃI�[�g�L�����u���[�V����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.28			sato			�V�K
// *************************************************************************/
PRIVATE void Dist_front_auto_calibration (void)
{
	USHORT i;
	ULONG ul_Lref = 0;
	ULONG ul_Rref = 0;
	
	/* �f�[�^�T���v�����O */
	for( i=0; i<DIST_REF_NUM; i++){			// 100��T���v�����O�������ϒl����̒l�Ƃ���B
		ul_Lref += st_sen[DIST_SEN_L_FRONT].s_now;
		ul_Rref += st_sen[DIST_SEN_R_FRONT].s_now;
		TIME_wait(1);
	}
	
	/* ��l�Z�o�i���ϒl�j */
	l_Lfront_DistRef = ul_Lref / DIST_REF_NUM ;		// ���x��100�{�ɂ���
	l_Rfront_DistRef = ul_Rref / DIST_REF_NUM ;
	
	st_sen[DIST_SEN_R_FRONT].s_ref       = l_Rfront_DistRef;
	st_sen[DIST_SEN_L_FRONT].s_ref       = l_Lfront_DistRef;
	st_sen[DIST_SEN_R_FRONT].s_limit     = l_Rfront_DistRef * R_FRONT_WALL_GAIN;
	st_sen[DIST_SEN_L_FRONT].s_limit     = l_Lfront_DistRef * L_FRONT_WALL_GAIN;
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = l_Rfront_DistRef * R_FRONT_CTRL;//�����l
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = l_Lfront_DistRef * L_FRONT_CTRL;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = st_sen[DIST_SEN_R_FRONT].s_noCtrl * R_FRONT_NOCTRL_GAIN;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = st_sen[DIST_SEN_L_FRONT].s_noCtrl * L_FRONT_NOCTRL_GAIN;
}

// *************************************************************************
//   �@�\		�F �I�[�g�L�����u���[�V����
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.28			sato			�V�K
// *************************************************************************/
PUBLIC void Dist_autocalibration(void)
{
	TIME_wait(2000);
	Dist_side_auto_calibration ();
	TIME_wait(100);
	MOT_goBlock_FinSpeed( 0.3, 0 );
	MOT_turn(MOT_R90);
	MOT_goHitBackWall();
	MOT_goBlock_FinSpeed( 0.3, 0 );
	TIME_wait(200);
	Dist_front_auto_calibration ();
	MOT_turn(MOT_L90);
	MOT_goHitBackWall();
//	printf("RF %5.2f LF %5.2f RS %5.2f LS %5.2 \n",st_sen[DIST_SEN_R_FRONT].s_ref,st_sen[DIST_SEN_L_FRONT].s_ref,st_sen[DIST_SEN_R_SIDE].s_ref,st_sen[DIST_SEN_L_SIDE].s_ref);
}

PUBLIC void calibration(void)
{
printf("RF %4df LF %4d RS %4d LS %4d \n",st_sen[DIST_SEN_R_FRONT].s_ref,st_sen[DIST_SEN_L_FRONT].s_ref,st_sen[DIST_SEN_R_SIDE].s_ref,st_sen[DIST_SEN_L_SIDE].s_ref);
printf("RF %4df LF %4d RS %4d LS %4d \n",st_sen[DIST_SEN_R_FRONT].s_limit,st_sen[DIST_SEN_L_FRONT].s_limit,st_sen[DIST_SEN_R_SIDE].s_limit,st_sen[DIST_SEN_L_SIDE].s_limit);
}