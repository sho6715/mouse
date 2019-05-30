// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F init�t�@�C���i���W�X�^�ݒ�t�@�C�����j
//   ����		�F �Ȃ�
//   ����		�F WORD = BYTE��2 LONG ��BYTE���S
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <init.h>							//���W�X�^


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




//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F �N���b�N�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_clock(void)
{


	SYSTEM.PRCR.WORD = 0xa50b;		//�N���b�N�\�[�X�I���̕ی�̉���

	SYSTEM.PLLCR.WORD = 0x0F00;		/* PLL ���{�~16 ����1���� (12.000MHz * 16 = 192MHz)*/
	SYSTEM.PLLCR2.BYTE = 0x00;		/* PLL ENABLE */
	
	SYSTEM.PLLWTCR.BYTE     = 0x0F;		/* 4194304cycle(Default) */

	
	// ICK   : 192/2 = 96MHz 		// �V�X�e���N���b�N CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz	 	// ���Ӄ��W���[���N���b�NA ETHERC�AEDMAC�ADEU
	// PCLKB : 192/4 = 48MHz 		// ���Ӄ��W���[���N���b�NB ��L�ȊO PCLKB=PCLK
/*	
	SYSTEM.SCKCR.BIT.FCK=0x02;		//FCLK MAX 50MHz  192/4
	SYSTEM.SCKCR.BIT.ICK=0x01;		//ICLK MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PSTOP1=0x01;		//BCLK �o�͒�~
	SYSTEM.SCKCR.BIT.PSTOP0=0x01;		//SDCLK �o�͒�~
	SYSTEM.SCKCR.BIT.BCK=0x02;		//BCLK MAX 100MHz ICLK�ȉ��ɂ���K�v������192/4
	SYSTEM.SCKCR.BIT.PCKA=0x01;		//PCLKA MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PCKB=0x02;		//PCLKB MAX 50MHz 192/4
	//��L�̐ݒ�ł͐�����clock�ݒ肪�ł��Ȃ����߉��L�̂悤�Ɉꊇ�Őݒ肷�邱��
*/
	SYSTEM.SCKCR.LONG = 0x21C21211;		//FCK1/4 ICK1/2 BCLK��~ SDCLK��~ BCK1/4 PCLKA1/2 PCLKB1/4
/*
	SYSTEM.SCKCR2.BIT.UCK=0x03;		//UCLK MAX 48MHz 192/4
	SYSTEM.SCKCR2.BIT.IEBCK=0x02;		//IECLK MAX 50MHz 192/4
*/
	SYSTEM.SCKCR2.WORD = 0x0032;		/* UCLK1/4 IEBCK1/4 */
	SYSTEM.BCKCR.BYTE = 0x01;		/* BCLK = 1/2 */
	
	SYSTEM.SCKCR3.WORD = 0x0400;		//PLL��H�I��

}

// *************************************************************************
//   �@�\		�F IO�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_io(void)
{
	/* ================== */
	/*  GPIO(�ėp���o��)  */
	/* ================== */
	/* �o�͒l */
	PORTE.PODR.BIT.B0			= 0;		//�|�[�gE-0�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTE.PODR.BIT.B1			= 0;		//�|�[�gE-1�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTE.PODR.BIT.B2			= 0;		//�|�[�gE-2�̏����o��0[V]�i�f�o�b�O�pLED�j
	PORTE.PODR.BIT.B3			= 0;		//�|�[�gE-3�̏����o��0[V]�i�f�o�b�O�pLED�j
	
	PORTE.PODR.BIT.B4			= 0;		//�|�[�gE-4�̏����o��0[V]�i�d���p��LED�j
	PORTE.PODR.BIT.B5			= 0;		//�|�[�gE-5�̏����o��0[V]�i�d���p��LED�j
	/*�Z���T*/
	PORTA.PODR.BIT.B1			= 0;		//�|�[�gA-1�̏����o��0[V]�i�Z���TLED�j
	PORTA.PODR.BIT.B3			= 0;		//�|�[�gA-3�̏����o��0[V]�i�Z���TLED�j
	PORTA.PODR.BIT.B4			= 0;		//�|�[�gA-4�̏����o��0[V]�i�Z���TLED�j
	PORTA.PODR.BIT.B6			= 0;		//�|�[�gA-6�̏����o��0[V]�i�Z���TLED�j
	//DCM
	PORTA.PODR.BIT.B0			= 0;		//PWM(TPU0)
	PORTB.PODR.BIT.B0			= 0;		//PWM(TPU3)
	PORTB.PODR.BIT.B1			= 0;		//AIN1
	PORTB.PODR.BIT.B3			= 0;		//AIN2
	PORTB.PODR.BIT.B5			= 0;		//BIN1
	PORTB.PODR.BIT.B6			= 0;		//BIN2
	PORTB.PODR.BIT.B7			= 0;		//STBY
	
//	PORT3.PODR.BIT.B			= 0;		//�W�����v�z���̑Ώ��i�V���[�g���ĂȂ��H�j���͂���������ď������Ȃ���΂��������ł�
	
	/* ���o�͐ݒ� */
	PORTE.PDR.BIT.B0			= 1;		//�|�[�gE-0���o�͂ɐݒ�
	PORTE.PDR.BIT.B1			= 1;		//�|�[�gE-1���o�͂ɐݒ�
	PORTE.PDR.BIT.B2			= 1;		//�|�[�gE-2���o�͂ɐݒ�
	PORTE.PDR.BIT.B3			= 1;		//�|�[�gE-3���o�͂ɐݒ�
	
	PORTE.PDR.BIT.B4			= 1;		//�|�[�gE-4���o�͂ɐݒ�
	PORTE.PDR.BIT.B5			= 1;		//�|�[�gE-5���o�͂ɐݒ�
	/*�Z���T*/
	PORTA.PDR.BIT.B1			= 1;		//�|�[�gA-1���o�͂ɐݒ�
	PORTA.PDR.BIT.B3			= 1;		//�|�[�gA-3���o�͂ɐݒ�
	PORTA.PDR.BIT.B4			= 1;		//�|�[�gA-4���o�͂ɐݒ�
	PORTA.PDR.BIT.B6			= 1;		//�|�[�gA-6���o�͂ɐݒ�
	//DCM
//	PORTA.PDR.BIT.B0			= 1;		//PWM(TPU0)
//	PORTB.PDR.BIT.B0			= 1;		//PWM(TPU3)
	PORTB.PDR.BIT.B1			= 1;		//AIN1
	PORTB.PDR.BIT.B3			= 1;		//AIN2
	PORTB.PDR.BIT.B5			= 1;		//BIN1
	PORTB.PDR.BIT.B6			= 1;		//BIN2
	PORTB.PDR.BIT.B7			= 1;		//STBY
	
	/* ���̓v���A�b�v�ݒ� */
	PORTC.PCR.BIT.B2			= 1;		// �|�[�gC-2�̓v���A�b�v���g�p	(�v�b�V���X�C�b�`�p)
	PORTC.PCR.BIT.B3			= 1;		// �|�[�gC-3�̓v���A�b�v���g�p	(�v�b�V���X�C�b�`�p)
	
	/* �|�[�g���̓f�[�^���W�X�^ */
	PORTC.PIDR.BIT.B2			= 1;		// �|�[�gC-6����̓|�[�g�Ɛڑ�����
	PORTC.PIDR.BIT.B3			= 1;		// �|�[�gC-7����̓|�[�g�Ɛڑ�����
	
	
	/* ========= */
	/*  A/D�ϊ�  */
	/* ========= */
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA17 	= 0;		// AD(12bit)�X�^���o�C����
	SYSTEM.PRCR.WORD = 0xA500;
	
	S12AD.ADCSR.BIT.CKS 		= 3;		// PCLK�ŕϊ�
	
	//TPU�G���R�[�_�p
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P14PFS.BIT.PSEL		= 4;
	MPC.P15PFS.BIT.PSEL		= 4;
	MPC.P16PFS.BIT.PSEL		= 4;
	MPC.P17PFS.BIT.PSEL		= 4;
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORT1.PMR.BIT.B4		= 1;
	PORT1.PMR.BIT.B5		= 1;
	PORT1.PMR.BIT.B6		= 1;
	PORT1.PMR.BIT.B7		= 1;
	
	//TPUPWM�p
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PA0PFS.BIT.PSEL		= 3;
	MPC.PB0PFS.BIT.PSEL		= 3;
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORTA.PMR.BIT.B0		= 1;
	PORTB.PMR.BIT.B0		= 1;
	
	
}

// *************************************************************************
//   �@�\		�F MTU�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PRIVATE void init_mtu(void)
{
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU0) 	= 0;						// 
	MSTP(MTU1) 	= 0;
	MSTP(MTU3) 	= 0;
	MSTP(MTU2)	= 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	MTU.TSTR.BYTE = 0;						//�^�C�}����X�g�b�v
	
	// -----------------------
	//  �V�X�e���p(MTU0)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
	MTU0.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU0.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU0.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU0.TGRA 					= 750 * 4;	// 1msec���Ɋ��荞��
	MTU0.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU0,TGIA0) = 1;	//���荞�ݗv�������� 
	IPR(MTU0,TGIA0) = 7;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU0,TGIA0)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST0 = 0;	//�^�C�}�X�g�b�v
	
	// -----------------------
	//  �o�b�e���[�p(MTU1)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
	MTU1.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU1.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU1.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU1.TGRA 					= 7500 * 4;	// 10msec���Ɋ��荞��
	MTU1.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU1,TGIA1) = 1;	//���荞�ݗv�������� 
	IPR(MTU1,TGIA1) = 6;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU1,TGIA1)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST1 = 0;	//�^�C�}�X�g�b�v
	
	// -----------------------
	//  �Z���T�p(MTU3)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
	MTU3.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU3.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU3.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU3.TGRA 					= 750;	// 250usec���Ɋ��荞��
	MTU3.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU3,TGIA3) = 1;	//���荞�ݗv�������� 
	IPR(MTU3,TGIA3) = 8;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU3,TGIA3)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST3 = 0;	//�^�C�}�X�g�b�v
	
	// -----------------------
	//  �Z���T�p(MTU2)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
/*	MTU2.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	MTU2.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 ��1�J�E���g
	MTU2.TIER.BIT.TGIEA			= 1;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�݋���
	MTU2.TGRA 					= 15000;	// 5msec���Ɋ��荞��
	MTU2.TCNT 					= 0;		// �^�C�}�N���A
	
	IEN(MTU2,TGIA2) = 1;	//���荞�ݗv�������� 
	IPR(MTU2,TGIA2) = 4;	//���荞�ݗD��x�����_�ɐݒ�
	IR(MTU2,TGIA2)	= 0;	//���荞�݃X�e�[�^�X�t���O���N���A
	
	MTU.TSTR.BIT.CST2 = 0;	//�^�C�}�X�g�b�v
*/	
}

// *************************************************************************
//   �@�\		�F TPU�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.07.03			sato			�V�K
// *************************************************************************/
PRIVATE void init_tpu(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(TPU1) 			= 0;
	MSTP(TPU2) 			= 0;
	MSTP(TPU0) 			= 0;
	MSTP(TPU3) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	// -----------------------
	//  �E���[�^�ʑ��v���p(TPU1)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
	TPU1.TCR.BIT.CCLR 			= 0;		// �R���y�A�}�b�`��TCNT�N���A�֎~
	TPU1.TIER.BIT.TGIEA			= 0;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�ݖ�����
	TPU1.TMDR.BIT.MD			= 7;		// �ʑ��v�����[�h4
	TPU1.TGRA 				= 0;		// �n�߂͖��ݒ�
	TPU1.TCNT 				= 32768;	// ������
	
	// -----------------------
	//  �����[�^�ʑ��v���p(TPU2)
	// -----------------------
	/* �^�C�}���荞�݂̐ݒ� */
	TPU2.TCR.BIT.CCLR 			= 0;		// �R���y�A�}�b�`��TCNT�N���A�֎~
	TPU2.TIER.BIT.TGIEA			= 0;		// TGRA�Ƃ̃R���y�A�}�b�`�Ŋ��荞�ݖ�����
	TPU2.TMDR.BIT.MD			= 7;		// �ʑ��v�����[�h4
	TPU2.TGRA 				= 0;		// �n�߂͖��ݒ�
	TPU2.TCNT 				= 32768;	// ������
	
	// -----------------------
	//  �E���[�^PWM�o�͗p(TPU0)
	// -----------------------
	TPU0.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	TPU0.TCR.BIT.CKEG			= 1;		// CKEG�����オ��G�b�W�ŃJ�E���g
	TPU0.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 ��1�J�E���g
	TPU0.TMDR.BIT.MD			= 2;		// PWM ���[�h 1
	TPU0.TIORH.BIT.IOA			= 2;		// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU0.TIORH.BIT.IOB			= 1;		// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	TPU0.TGRA 				= 600;		// ����(50usec)
	TPU0.TGRB 				= 300;		// onDuty
	TPU0.TCNT 				= 0;		// �^�C�}�N���A
	
	// -----------------------
	//  �����[�^PWM�o�͗p(TPU3)
	// -----------------------
	TPU3.TCR.BIT.CCLR 			= 1;		// TGRA�̃R���y�A�}�b�`��TCNT�N���A
	TPU3.TCR.BIT.CKEG			= 1;		// CKEG�����オ��G�b�W�ŃJ�E���g
	TPU3.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 ��1�J�E���g
	TPU3.TMDR.BIT.MD			= 2;		// PWM ���[�h 1
	TPU3.TIORH.BIT.IOA			= 2;		// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	TPU3.TIORH.BIT.IOB			= 1;		// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	TPU3.TGRA 				= 600;		// ����(50usec)
	TPU3.TGRB 				= 300;		// onDuty
	TPU3.TCNT 				= 0;		// �^�C�}�N���A
	
}

// *************************************************************************
//   �@�\		�F SCI�̃��W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.07.03			sato			�V�K
// *************************************************************************/
PRIVATE void init_sci(void)
{
/*	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE		= 0x00;		// TE=0�ARE=0�ACKE1=0
	while (0x00 != (SCI1.SCR.BYTE & 0xF0));	//���荞�ݗv�����֎~�����܂ő҂�
	PORT2.PODR.BIT.B6 = 1;			//TXD��Dirction�̐؂�ւ���̒l��high
	PORT2.PDR.BIT.B6 = 1;			//�o�͂ɐݒ�
	PORT3.PDR.BIT.B0 = 0;			//���͂ɐݒ�
	PORT2.PMR.BIT.B6 = 0;			//�ėp�|�[�g�ɐݒ�
	PORT3.PMR.BIT.B0 = 0;			//�ėp�|�[�g�ɐݒ�
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//���Ӌ@�\(RXD1)�Ƃ��Ďg�p
	SCI1.SCR.BIT.CKE = 0;
	SCI1.SMR.BYTE = 0x00;			//1stopbit parity�Ȃ� 8bit ��������
	SCI1.SCMR.BYTE = 0xF2;			//S=32clock
	SCI1.SEMR.BYTE = 0x00;
  	SCI1.BRR =38; 				//@48MHz 38400bps
  	SCI1.SCR.BYTE =0xF0;			//���M���荞�݋֎~
	PORT2.PMR.BIT.B6 = 1;			//���Ӌ@�\(TXD1)�Ƃ��Ďg�p
	SCI1.SCR.BIT.TE = 1;
	SCI1.SCR.BIT.RE = 1;
	
	IEN(SCI1,RXI1) 		= 1;
	IPR(SCI1,RXI1) 		= 2;
	IR(SCI1,RXI1) 		= 0;
*/
//	IR(SCI1,TXI1) 		= 0;
	
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE	= 0x00;
	while(0x00 != (SCI1.SCR.BYTE & 0xF0));
	SCI1.SMR.BYTE	= 0x00;
//	PORT2.PODR.BIT.B6 = 1;			//TXD��Dirction�̐؂�ւ���̒l��high
//	PORT2.PDR.BIT.B6 = 1;			//�o�͂ɐݒ�
//	PORT3.PDR.BIT.B0 = 0;			//���͂ɐݒ�
//	PORT2.PMR.BIT.B6 = 0;			//�ėp�|�[�g�ɐݒ�
//	PORT3.PMR.BIT.B0 = 0;			//�ėp�|�[�g�ɐݒ�
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//���Ӌ@�\(RXD1)�Ƃ��Ďg�p
	PORT2.PMR.BIT.B6 = 1;			//���Ӌ@�\(TXD1)�Ƃ��Ďg�p
	SCI1.BRR	= 78;
	SCI1.SEMR.BIT.ABCS = 1;
	SCI1.SCR.BYTE	=0x30;

	IEN(SCI1,RXI1) 		= 1;
	IEN(SCI1,TXI1) 		= 1;
	ICU.IPR[217].BIT.IPR	= 5;

}

// *************************************************************************
//   �@�\		�F SPI�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PUBLIC void init_spi(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(RSPI0) 		= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	RSPI0.SPCR.BYTE = 0x00;
	
//	PORTC.PMR.BIT.B4	= 1;
	PORTC.PMR.BIT.B5	= 1;
	PORTC.PMR.BIT.B6	= 1;
	PORTC.PMR.BIT.B7	= 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
//	MPC.PC4PFS.BYTE = 0x0D;
	MPC.PC5PFS.BYTE = 0x0D;  /* RSPI RSPCKA */       
	MPC.PC6PFS.BYTE = 0x0D;  /* RSPI MOSIA  */
	MPC.PC7PFS.BYTE = 0x0D;  /* RSPI MISOA  */
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;
	PORTC.PODR.BIT.B4 = 1;
	PORTC.PDR.BIT.B4 = 1;
	
	RSPI0.SPBR		= 2;
	RSPI0.SPCMD0.WORD	= 0x0783;	
	
	RSPI0.SPCR.BYTE = 0xF8;

}

// *************************************************************************
//   �@�\		�F ���W�X�^�ݒ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.06.30			sato			�V�K
// *************************************************************************/
PUBLIC void CPU_init(void)
{
	init_clock();
	init_io();
	init_mtu();
	init_tpu();
	init_sci();
	init_spi();
}
