// *************************************************************************
//   ���{�b�g��		�F AEGIS
//   �T�v		�F DataFlash�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.25			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <search2.h>
#include <parameters.h>
#include <DataFlash.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define BASEADDR	0x00100000
#define FLASHSIZE	0x7fff		//32kB

#define BLOCKSIZE	32		//bytes
#define BLOCKCOUNT	1024		//count

#define write_byte(A,D)	*(UCHAR *)(A)=(D)
#define write_word(A,D)	*(USHORT *)(A)=(D)


//**************************************************
// �񋓑́ienum�j
//**************************************************


//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************
const UCHAR *BaseAddr = (const unsigned char *)BASEADDR;

static int isErr = 0;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F map�X�g���[�W��������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.25			sato		�V�K
// *************************************************************************/
void map_write(void)
{
	short i;
	unsigned short *map_add;
	map_add = (unsigned short*)g_sysMap;
	
	//DataFlash�C���[�X
	for(i=0;i<8;i++){
		erase((ULONG)(MAP_ADD+i*32));
	}
	//�}�b�v�f�[�^��DataFlash�ɏ�������
	for(i=0;i<128;i++){
		write_eeflash((ULONG)(MAP_ADD+i*2),map_add);
		map_add++;
	}
}

// *************************************************************************
//   �@�\		�F �}�b�v�f�[�^��RAM�ɃR�s�[
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.25			sato		�V�K
// *************************************************************************/
void map_copy(void)
{
	short i;
	unsigned short *map_add;
	map_add = (unsigned short*)&g_sysMap;

	//�}�b�v�f�[�^��RAM�ɃR�s�[
	for(i=0;i<128;i++){
//		*map_add = *(unsigned short *)(MAP_ADD+i*2);
		Flash_read((unsigned short *)(MAP_ADD+i*2),map_add);
		map_add++;
//		printf("map_add %x\n\r",map_add);
	}
}

// *************************************************************************
//   �@�\		�F map�C���[�Y
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.28			sato		�V�K
// *************************************************************************/
void map_erase(void)
{
	short i;
	
	//DataFlash�C���[�X
	for(i=0;i<8;i++){
		erase((ULONG)(MAP_ADD+i*32));
	}
}

// *************************************************************************
//   �@�\		�F FCU�����Z�b�g����B�G���[�����p
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.25			sato		�V�K
// *************************************************************************/
static void fcu_reset(void)
{
	FLASH.FRESETR.BIT.FRESET = 1;
	TIME_wait(2);
	FLASH.FRESETR.BIT.FRESET = 0;
}


//FCU�����҂� timeout�Ƀ^�C���A�E�g�J�E���g[mS]��ݒ�
static void wait_fcuRdy(int timeout)
{
	int isTimeout = 0;
	
	TIME_wait(timeout);
	
	if(FLASH.FSTATR0.BIT.FRDY == 0)//P/E������
	{
		isTimeout = 1;
//		//SCI_printf("FCU time out\n\r");
	}
	
	//�^�C���A�E�g���Ă����烊�Z�b�g
	if(isTimeout == 1)
	{
		fcu_reset();
	}
	
	return;
}


//�G���[���m�F���A�G���[������ΏC������
static void check_error(void)
{
	int iserr = 0;
	
	iserr |= FLASH.FSTATR0.BIT.ILGLERR;//FCU�͕s���ȃR�}���h��s����E2�f�[�^�t���b�V���A�N�Z�X�����o
	iserr |= FLASH.FSTATR0.BIT.ERSERR;//�C���[�X���ɃG���[����
	iserr |= FLASH.FSTATR0.BIT.PRGERR;//�v���O�������ɃG���[����
	
	//printf("[%s]",tmp_source);
	if(iserr == 0)
	{
//		printf("No error\n\r");
		return;	//no error
	}
	
	isErr = 1;
//	_LED(0x07);
	LED4 = LED4_ALL_ON;
//	//SCI_printf("FCU Error\n\r");
//	printf("FCU Error\n\r");
	if(FLASH.FSTATR0.BIT.ILGLERR == 1)
	{
//		printf("FSTATR0:%02X\nFSTATR1:%02X\nFASTAT:%02X\n\n", FLASH.FSTATR0.BYTE, FLASH.FSTATR1.BYTE, FLASH.FASTAT.BYTE);
		
		if(FLASH.FASTAT.BYTE != 0x10)
		{
			FLASH.FASTAT.BYTE = 0x10;//�t���O���N���A
		}
	}
	
	write_byte(BaseAddr, 0x50);	//status clear
}



//�w�肵���u���b�N���C���[�X����
//P1888 �}47.14 ROM/E2�f�[�^�t���b�V���C���[�X���@ 
//�C���[�X��32�o�C�g
void erase(ULONG addr)
{
	volatile UCHAR *a = (UCHAR *)addr;
	transition_pe();
		
		*a = 0x20;
		*a = 0xD0;
		wait_fcuRdy(5);
		check_error();
	
	transition_read();
}


//�w�肵���̈�ɏ�������
//P1887 �}47.13 ROM/E2�f�[�^�t���b�V���ւ̃v���O�������@ 
void write_eeflash(ULONG addr, USHORT *data)
{
	volatile USHORT *a = (USHORT *)addr;
	volatile UCHAR *b = (UCHAR *)addr;
	
	transition_pe();
	{
		*b = 0xE8;
		*b = 0x01;
		*a = *data;
		*b = 0xD0;	
		wait_fcuRdy(3);
		check_error();
	}
	transition_read();
	
}

char blank_check(ULONG addr)
{
	volatile USHORT *a = (USHORT*)addr;
	
	transition_pe();
	
	FLASH.FMODR.BIT.FRDMD = 1;
	FLASH.DFLBCCNT.WORD = 0x0000;
	*a = 0x71;
	*a = 0xD0;
	wait_fcuRdy(2);
	check_error();
	transition_read();
	
	return FLASH.DFLBCSTAT.BIT.BCST;
	
}

//FCU��ǂݍ��݃��[�h�ɑJ�ڂ�����
static void transition_read(void)
{
	FLASH.FENTRYR.WORD = 0xAA00;	//E2Flash P/E mode

	while(FLASH.FENTRYR.WORD != 0x0000);
	
	/* Flash write/erase disabled */
	FLASH.FWEPROR.BYTE = 0x02;
}

//FCU��P/E���[�h�ɑJ�ڂ�����
static void transition_pe(void)
{
	
	FLASH.FENTRYR.WORD = 0xAA00;
	while(0x0000 != FLASH.FENTRYR.WORD);
	
	FLASH.FENTRYR.WORD = 0xAA80;	//E2Flash P/E mode
	check_error();
	
	FLASH.FWEPROR.BYTE = 0x01;	//�t���b�V��P/E�v���e�N�g���W�X�^
					//P/E ���b�N�r�b�g��P/E ���b�N�r�b�g�̓ǂݏo���A�u�����N�`�F�b�N�̋���
}

//DataFlash������������
void hw_dflash_init(void)
{
	unsigned int i;
	
	FLASH.DFLRE0.WORD = 0x2DFF;	// block read enable E2�f�[�^�t���b�V���ǂݏo�������W�X�^
	FLASH.DFLRE1.WORD = 0xD2FF;	// block read enable E2�f�[�^�t���b�V���ǂݏo�������W�X�^
	FLASH.DFLWE0.WORD = 0x1EFF;	// block P/E enable E2�f�[�^�t���b�V��P/E�����W�X�^
	FLASH.DFLWE1.WORD = 0xE1FF;	// block P/E enable E2�f�[�^�t���b�V��P/E�����W�X�^
//P1883 �}47.10 P/E�����̊T���t���[
//FCU RAM�ւ̃t�@�[���E�F�A�]��
	if(FLASH.FENTRYR.WORD != 0x0000)
	{
		FLASH.FENTRYR.WORD = 0xAA00;//FCU��~
	}
	FLASH.FCURAME.WORD=0xC401;
	
	for(i=0;i<8192;i++)
	{
		*(unsigned char *)(0x007F8000+i) = *(unsigned char *)(0xFEFFE000+i);
	}
	
	transition_pe();
	FLASH.PCKAR.BIT.PCKA = 48;	//FlashIF clock is 48MHz
	
	//���ӃN���b�N�ʒm�R�}���h
	write_byte(BaseAddr, 0xE9);
	write_byte(BaseAddr, 0x03);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_byte(BaseAddr, 0xD0);
	
	wait_fcuRdy(2);//63us tPCKA��63us�ł悢�B�^�C�}�[�Ƃ���1ms������ł�����g�p���Ă��邽��1ms�ɂ��Ă���
	transition_read();


}	

PUBLIC void flash_test(USHORT r)
{
	USHORT ram = r;
	USHORT *read;
	erase((ULONG)0x00101000);
	write_eeflash((ULONG)0x00101000, &ram);
	TIME_wait(1000);
			
	*read = *(USHORT *)0x00101000;
	printf("testdata %d\r\n",read);
	
}

// *************************************************************************
//   �@�\		�F �f�[�^�t���b�V�����[�h�֐�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.26			sato		�V�K
// *************************************************************************/
PUBLIC void Flash_read(USHORT *add, USHORT *data)
{
	USHORT *read;
	if(FLASH.FENTRYR.WORD&0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;
	}
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
			
	*read = *(USHORT *)add;
	*data = *read;
//	printf("%d",read);
	
}
