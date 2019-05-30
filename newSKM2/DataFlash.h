// *************************************************************************
//   ���{�b�g��		�F AEGIS
//   �T�v		�F DataFlash�̃w�b�_�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.25			sato			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���}�~
#ifndef _DATAFLASH_H
#define _DATAFLASH_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <init.h>
#include <parameters.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define REF_SEN_R_ADD 0x00100000
#define REF_SEN_L_ADD 0x00100002
#define TH_SEN_R_ADD  0x00100004
#define TH_SEN_L_ADD  0x00100006
#define TH_SEN_FR_ADD 0x00100008
#define TH_SEN_FL_ADD 0x0010000a

#define MAP_ADD		  0x00100800

//**************************************************
// �񋓑́ienum�j
//**************************************************


//**************************************************
// �\���́istruct�j
//**************************************************


//**************************************************
// �O���[�o���ϐ�
//**************************************************
void map_write(void);
void map_erase(void);
void hw_dflash_init(void);
void write_eeflash(ULONG addr, USHORT *data);
void erase(ULONG addr);
char blank_check(ULONG addr);
void map_copy(void);
static void transition_read(void);
static void transition_pe(void);
PUBLIC void flash_test(USHORT r);
PUBLIC void Flash_read(USHORT *add, USHORT *data);


//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

#endif