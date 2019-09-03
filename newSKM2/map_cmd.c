// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����mode�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.08.25			sato		�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/


//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <hal.h>						// HAL
#include <stdio.h>
#include <search2.h>
#include <parameters.h>
#include <init.h>
#include <hal_dist.h>
#include <map_cmd.h>

//**************************************************
// ��`�idefine�j
//**************************************************
#define LIST_NUM			( 4096 )				// �R�}���h���s�̃��X�g��


//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
/* �V�~�����[�V���� */
typedef struct{
	enMAP_CMD	en_cmd;			// �R�}���h
	FLOAT		f_x0_x1;		// [0]/[1]��X���W���Z�l
	FLOAT		f_y0_y1;		// [0]/[1]��y���W���Z�l
	FLOAT		f_x2_x3;		// [2]/[3]��X���W���Z�l
	FLOAT		f_y2_y3;		// [2]/[3]��y���W���Z�l
	FLOAT		f_x4_x5;		// [4]/[5]��X���W���Z�l
	FLOAT		f_y4_y5;		// [4]/[5]��y���W���Z�l
	FLOAT		f_x6_x7;		// [6]/[7]��X���W���Z�l
	FLOAT		f_y6_y7;		// [6]/[7]��y���W���Z�l
	SHORT		s_dir;			// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
}stMAP_SIM;


//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �R�}���h���X�g */
PUBLIC	UCHAR		dcom[LIST_NUM];					// ���n�M����p
PUBLIC	UCHAR		scom[LIST_NUM];					// �X�����[���p
PUBLIC	UCHAR		tcom[LIST_NUM];					// �΂ߑ��s�p
PRIVATE	USHORT		us_totalCmd;					// �g�[�^���R�}���h��

PRIVATE	FLOAT		f_PosX;							// X���W
PRIVATE	FLOAT		f_PosY;							// Y���W
PRIVATE	SHORT		s_PosDir;						// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j

/* �R�}���h�ɉ��������W�X�V�f�[�^ */
PRIVATE CONST stMAP_SIM st_PosData[] = {
	
	//	�R�}���h	[0]/[1]��X	[0]/[1]��Y	[2]/[3]��X	[2]/[3]��Y	[4]/[5]��X	[4]/[5]��Y	[6]/[7]��X	[6]/[7]��Y	����
	{ R90,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [0]
	{ L90,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [1]
	{ R90S,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [2]
	{ L90S,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [3]
	{ RS45N,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		+1 },		// [4]
	{ LS45N,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-1 },		// [5]
	{ RS135N,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+3 },		// [6]
	{ LS135N,		-0.75,		0.25,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-3 },		// [7]
	{ RN45S,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+1 },		// [8]
	{ LN45S,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		-1 },		// [9]
	{ RN135S,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		0.25,		0.75,		+3 },		// [10]
	{ LN135S,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-3 },		// [11]
	{ RN90N,		0.5,		0,			0,			-0.5,		-0.5,		0,			0,			0.5,		+2 },		// [12]
	{ LN90N,		0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			-2 },		// [13]
	{ GO1,			0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			0  },		// [14]
	{ NGO1,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		0  },		// [15]
	{ MAP_CMD_MAX,	0,			0,			0,			0,			0,			0,			0,			0,			0  },
};

PRIVATE FLOAT f_LogPosX[30];
PRIVATE FLOAT f_LogPosY[30];
PRIVATE USHORT us_LogIndex = 0;
PRIVATE USHORT us_LogWallCut[30];
PRIVATE USHORT us_LogIndexWallCut = 0;



//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************


// *************************************************************************
//   �@�\		�F �ʒu���X�V����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_refPos( UCHAR uc_cmd )
{
	UCHAR uc_index = 0;			// �e�[�u���̃C���f�b�N�X�ԍ�
	
	/* ------------------------------------------ */
	/*  �R�}���h����e�[�u���̃C���f�b�N�X���擾  */
	/* ------------------------------------------ */
	/* ���i */
	if( ( uc_cmd <=  GO71 ) && ( uc_cmd >=  GO1) ){
		
		uc_index = 14;		// st_PosData�e�[�u���̒��i�̃C���f�b�N�X�ԍ�
	}
	/* �΂ߒ��i */
	else if( ( uc_cmd <=  NGO71 ) && ( uc_cmd >=  NGO1) ){
		
		uc_index = 15;		// st_PosData�e�[�u���̎΂ߒ��i�̃C���f�b�N�X�ԍ�
	}
	/* ���̑��̃R�}���h */
	else{
		while(1){
			
			if( st_PosData[uc_index].en_cmd == uc_cmd )      break;			// �R�}���h����
			if( st_PosData[uc_index].en_cmd == MAP_CMD_MAX ) return;		// �R�}���h������
			uc_index++;
		}
	}
	
	/* �ʒu�X�V */
	switch( s_PosDir ){
		
		/* [0]�k [1]�k�� */
		case 0:
		case 1:
		
			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y0_y1 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y0_y1 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x0_x1;
				f_PosY += st_PosData[uc_index].f_y0_y1;
			}
			break;
		
		/* [2]�� [3]�쓌 */
		case 2:
		case 3:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y2_y3 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y2_y3 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x2_x3;
				f_PosY += st_PosData[uc_index].f_y2_y3;
			}
			break;

		/* [4]�� [5]�쐼 */
		case 4:
		case 5:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y4_y5 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y4_y5 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x4_x5;
				f_PosY += st_PosData[uc_index].f_y4_y5;
			}
			break;

		/* [6]�� [7]�k�� */
		case 6:
		case 7:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y6_y7 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y6_y7 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x6_x7;
				f_PosY += st_PosData[uc_index].f_y6_y7;
			}
			break;
	}
	
	/* �i�s�����X�V */
	s_PosDir += st_PosData[uc_index].s_dir;
	if( s_PosDir < 0 ) s_PosDir += 8;				// [0]�`[7]�ɂ�����
	else if( s_PosDir > 7 ) s_PosDir -= 8;
	
	f_LogPosX[us_LogIndex] = f_PosX;
	f_LogPosY[us_LogIndex] = f_PosY;
	
	us_LogIndex++;
	us_LogIndex %= 30;
}

// *************************************************************************
//   �@�\		�F �R�[�i�O�ɕǂ���������؂�ڕ␳���s���ݒ������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE BOOL MAP_setWallCut( UCHAR uc_cmd )
{
	UCHAR uc_val = 0;			// 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ�ǂ���j
	UCHAR uc_valPrev = 0;		// 2���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ�ǂ���j
	BOOL bl_wallCut = FALSE;
	
	/* �ʒu�X�V */
	switch( uc_cmd ){
		
		case R90S:
		case RS135N:
			
			/* 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ炠��j */
			// s_PosDir�F�i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
			switch( s_PosDir ){
				
				/* ����Ő��񂷂�̂ŁA������O���ǂ̗L���𒲂ׂ������W�ƂȂ�i���ӁFg_sysMap��2�����z��ł��j */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x02;		// �k�������Ă���̂œ����̕ǂ����邩
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x02;		// �k�������Ă���̂œ����̕ǂ����邩
					break;	
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x04;		// ���������Ă���̂œ쑤�̕ǂ����邩
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x04;		// ���������Ă���̂œ쑤�̕ǂ����邩
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x08;		// ��������Ă���̂Ő����̕ǂ����邩
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x08;		// ��������Ă���̂Ő����̕ǂ����邩
					break;
				case 6:
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x01;		// ���������Ă���̂Ŗk���̕ǂ����邩
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x01;		// ���������Ă���̂Ŗk���̕ǂ����邩
					break;
			}
			/* �ǂ����邽�ߕǐ؂�␳���s�� */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_RIGHT );		// �ǐ؂�␳�����{����
				bl_wallCut = TRUE;
			}
			break;
			
		case L90S:
		case LS135N:
			/* 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ炠��j */
			// s_PosDir�F�i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
			switch( s_PosDir ){
				
				/* ����Ő��񂷂�̂ŁA������O���ǂ̗L���𒲂ׂ������W�ƂȂ�i���ӁFg_sysMap��2�����z��ł��j */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x08;			// �k�������Ă���̂Ő����̕ǂ����邩
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x08;			// �k�������Ă���̂Ő����̕ǂ����邩
					break;
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x01;			// ���������Ă���̂Ŗk���̕ǂ����邩
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x01;			// ���������Ă���̂Ŗk���̕ǂ����邩
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x02;			// ��������Ă���̂œ����̕ǂ����邩
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x02;			// ��������Ă���̂œ����̕ǂ����邩
					break;
				case 6: 
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x04;			// ���������Ă���̂œ쑤�̕ǂ����邩
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x04;			// ���������Ă���̂œ쑤�̕ǂ����邩
					break;
			}
			/* �ǂ����邽�ߕǐ؂�␳���s�� */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_LEFT );		// �ǐ؂�␳�����{����
				bl_wallCut = TRUE;
			}
			break;
			
		default:
			break;
	}
	
	return bl_wallCut;
}

// *************************************************************************
//   �@�\		�F �R�}���h���s�p�̍��W�ʒu��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PROTECTED void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	f_PosX   = (FLOAT)uc_x;
	f_PosX   = (FLOAT)uc_y;
	s_PosDir = (SHORT)(en_dir * 2);	// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j�A2�{����ƒ��x�l�����v����
	
}

// *************************************************************************
//   �@�\		�F ���H�R�}���h�f�[�^��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_showCmdLog( void )
{
	USHORT i=0;
	
	/* ���M�n����R�}���h */
	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
	}
	i=0;
	
	/* �X�����[���R�}���h */
	while(1){
		
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
		if( scom[i] == CEND ) break;
		i++;
	}
	i=0;

	/* �΂ߑ��s�R�}���h */
	while(1){
		
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
	}

}

// *************************************************************************
//   �@�\		�F ���M�n����R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_makeCmdList( 
	UCHAR uc_staX,					///< [in] �J�nX���W
	UCHAR uc_staY,					///< [in] �J�nY���W
	enMAP_HEAD_DIR en_staDir,		///< [in] �J�n���̕���
	UCHAR uc_endX,					///< [in] �I��X���W
	UCHAR uc_endY,					///< [in] �I��Y���W
	enMAP_HEAD_DIR* en_endDir		///< [out] �I�����̕���
){
	UCHAR			uc_goStep;									// �O�i�̃X�e�b�v��
	USHORT			us_high;									// �������̍���
	USHORT			us_pt;										// �R�}���h�|�C���^
	enMAP_HEAD_DIR	en_nowDir;									// ���݃}�E�X�̌����Ă����Ε���
	enMAP_HEAD_DIR	en_tempDir;									// ���Ε���
//	USHORT			i;											// roop
	
	/* �O�i�X�e�b�v�������������� */
	uc_goStep = 0;
	us_pt = 0;

	/* ���H��񂩂�R�}���h�쐬 */
	while(1){	
		us_high = us_cmap[uc_staY][uc_staX]-1;
		if (en_staDir == NORTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else   while(1);
		}else if (en_staDir == EAST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else   while(1);
		}else if (en_staDir == SOUTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else   while(1);
		}else if (en_staDir == WEST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else   while(1);
		}
		
		en_tempDir = (enMAP_HEAD_DIR)( (en_nowDir - en_staDir) & (enMAP_HEAD_DIR)3 );		// �����X�V
		en_staDir = en_nowDir;

		if (en_tempDir == NORTH){
			uc_goStep = uc_goStep + 2;
		}
		else if (en_tempDir == EAST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R90;
			uc_goStep = 2;
			us_pt++;
		}
		else if (en_tempDir == WEST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = L90;
			uc_goStep = 2;
			us_pt++;
		}
		else{
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R180;
			uc_goStep = 2;
			us_pt++;
		}

		if      (en_nowDir == NORTH) uc_staY = uc_staY + 1;
		else if (en_nowDir == EAST) uc_staX = uc_staX + 1;
		else if (en_nowDir == SOUTH) uc_staY = uc_staY - 1;
		else if (en_nowDir == WEST) uc_staX = uc_staX - 1;
		
		en_staDir = en_nowDir;
		
		if ((uc_staX == uc_endX) &&(uc_staY == uc_endY)) break;
	}
	
	/* ���n�M����p�̃R�}���h���X�g�쐬 */
	dcom[us_pt] = uc_goStep;
	dcom[++us_pt] = STOP;
	dcom[++us_pt] = CEND;
	us_totalCmd = us_pt+1;			// �R�}���h����


	/* �ŏI�I�Ɍ����Ă������ */
	*en_endDir = en_staDir;
	
#if 0
	/* debug */
	for( i = 0; i < us_totalCmd; i++){
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
	}
#endif
}

// *************************************************************************
//   �@�\		�F �X�����[������R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_makeSuraCmdList( void )
{
	USHORT dcom_temp[4096];			// ����撴�M����R�}���h���X�g
	USHORT i=0,j=0;					// roop
	
	/* ���n�M����R�}���h���R�s�[ */
	for( i=0; i<us_totalCmd; i++ ){
		dcom_temp[i] = dcom[i];
	}

	i = 0;

	/* �z�񂪐���R�}���h�����`�F�b�N */
	while(1)
	{
		if( dcom_temp[i] == R90 ){		// �E90��
			dcom_temp[i-1] -= 1;		// 1��O������
			dcom_temp[i+1] -= 1;		// 1��O������
			dcom_temp[i] = R90S;		// �E�X�����[��90��
		}
		else if( dcom_temp[i] == L90 ){	// ��90��
			dcom_temp[i-1] -= 1;		// 1��O������
			dcom_temp[i+1] -= 1;		// 1��O������
			dcom_temp[i] = L90S;		// ���X�����[��90��
		}
		else{
			if( dcom_temp[i] == CEND ){
				break;
			}
		}
		i++;
	}

	i = j = 0;

	/* �X�����[���R�}���h�ϊ� */
	while(1)
	{
		if( dcom_temp[i+1] == CEND ){
			scom[j] = STOP;
			scom[j+1] = CEND;
			break;
		}
		else
		{
			/* �f�[�^���X�g�b�v�R�}���h�������� */
			if( dcom_temp[i] == 0 ){
				i++;
			}
			
			scom[j] = dcom_temp[i];
			
			i++;
			j++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
	}
#endif

}

// *************************************************************************
//   �@�\		�F �΂ߓ���R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.10.23			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_makeSkewCmdList( void )
{
	USHORT	scom_temp[4096];			// ����撴�M����R�}���h���X�g
	USHORT	i;							// roop
	USHORT	c1, c2, c3, c4;				// �v�Z�p
	USHORT	x;
	USHORT	ct_n=0, ct_st=0;
	USHORT	flag = 3;					//	�΂ߑ��s�p�o�b�t�@  0:�����R�}���h�@1:�΂�  2:S135N �� N135S  3:���i
	
	/* ���n�M����R�}���h���R�s�[ */
	for( i=0; i<us_totalCmd; i++ )
	{
		scom_temp[i] = scom[i];
	}

	i=0;

	/* �z�񂪐���R�}���h�����`�F�b�N */
	while(1)
	{
		c1 = scom_temp[ct_st];
		c2 = scom_temp[ct_st+1];
		c3 = scom_temp[ct_st+2];
		c4 = scom_temp[ct_st+3];

		//	���i �� �E45�x �� �΂�
		if( (c1<=GO32) && (c2==R90S) && (c3==L90S) && (ct_st != 0) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	�O�̕����R�}���h�ɂ���Ē�����Ԃ������Ȃ��ꍇ
			tcom[ ct_n++ ] = RS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 0;
		}
		//	���i �� ��45�x �� �΂�
		else if( (c1<=GO32) && (c2==L90S) && (c3==R90S) && (ct_st != 0) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	�O�̕����R�}���h�ɂ���Ē�����Ԃ������Ȃ��ꍇ
			tcom[ ct_n++ ] = LS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 0;
		}

		//	���i �� �E90�x �� ���i
		else if( (c1<=GO32) && (c2==R90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = R90S;
			ct_st += 2;
			flag = 3;		//	���i
		}
		//	���i �� ��90�x �� ���i
		else if( (c1<=GO32) && (c2==L90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = L90S;
			ct_st += 2;
			flag = 3;		//	���i
		}
		//	���i �� �E135�x �� �΂�
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = RS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 2;
		}
		//	���i �� ��135�x �� �΂�
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = LS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 2;
		}

		//	���i �� �E180�x �� ���i
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = R90S;
			tcom[ ct_n++ ] = R90S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	���i �� ��180�x �� ���i
		else if( (c1<=GO32) && (c2==L90S) && (c2==L90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = L90S;
			tcom[ ct_n++ ] = L90S;
			ct_st += 3;
			flag = 3;		//	���i
		}

		//	�΂� �� �E45�x �� ���i
		else if( (c1==R90S) && (c2<=GO32)  && (flag != 3 ) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = x;
			tcom[ ct_n++ ] = RN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	������Ԃ�1���炷
			ct_st ++;
			flag = 3;		//	���i
		}
		//	�΂� �� ��45�x �� ���i
		else if( (c1==L90S) && (c2<=GO32)  && (flag != 3 ) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = x;
			tcom[ ct_n++ ] = LN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	������Ԃ�1���炷
			ct_st ++;
			flag = 3;		//	���i
		}
		//	�΂� �� �E90�x �� �΂�
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4==L90S)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����RN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 1;
		}
		//	�΂� �� ��90�x �� �΂�
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4==R90S)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 1;
		}
		//	�΂� �� �E135�x �� ���i
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4<=GO32)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN135S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	�΂� �� ��135�x �� ���i
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4<=GO32)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN135S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	�΂� �� �΂�
		else if( (c1==R90S) && (c2==L90S) && ( (c3==R90S) || (c3==L90S) || ( c3<=GO32 ) ) && (flag != 3 ) )
		{
			x++;
			ct_st ++;

			flag = 1;		//	�΂ߑ��s�o�b�t�@����
		}
		else if( (c1==L90S) && (c2==R90S) && ( (c3==L90S) || (c3==R90S) || ( c3<=GO32 ) ) && (flag != 3 ) )
		{
			//	�R�}���h�o��
			x++;
			ct_st ++;

			flag = 1;		//	�΂ߑ��s�o�b�t�@����
		}
		else
		{
			tcom[ ct_n ] = scom_temp[ct_st];
			if( tcom[ ct_n ] == CEND ) break;
			ct_st ++;
			ct_n ++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
	}
#endif

}

// *************************************************************************
//   �@�\		�F �R�}���h���s���W���[��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_drive( enMAP_DRIVE_TYPE en_driveType )
{
	USHORT			us_rp = 0;				// ���݂̓ǂݍ��݈ʒu
	enMOT_TURN_CMD 		en_type;
	BOOL			bl_isWallCut = FALSE;
	
	/* ���M���񃂁[�h*/
	if( en_driveType == MAP_DRIVE_TURN )
	{
		while(1)
		{
			if ( dcom[us_rp] == CEND  ) break;								//	�R�}���h�I��
			
			else if ( dcom[us_rp] == STOP  ){
			 	CTRL_stop();			// �����~
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
			}
			else if ( ( dcom[us_rp] <=  GO71 ) && ( dcom[us_rp] >=  GO1) )
			{
				MOT_goBlock_FinSpeed( (FLOAT)dcom[us_rp]*0.5f, 0 );		// �������s�R�}���h�A����ԑO�i��ɒ�~
			}
			else{
				
				if( dcom[us_rp] == R90 ) en_type = MOT_R90;
				else 					 en_type = MOT_L90;
				
				TIME_wait(500);
				MOT_turn( en_type );		//	����
				TIME_wait(500);
			}
			us_rp++;
			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot(DCM_R);
				DCM_brakeMot(DCM_L);
				break;
			}
			
		}
	 	CTRL_stop();			// �����~
		DCM_brakeMot( DCM_R );		// �u���[�L
		DCM_brakeMot( DCM_L );		// �u���[�L
	}
	/* �X�����[�����[�h */
	else if( en_driveType == MAP_DRIVE_SURA )
	{
		while(1)
		{
			MAP_refPos( scom[us_rp] );									// ���s�����R�}���h���I�������ʒu�ɍX�V

			if ( scom[us_rp] == CEND  ) break;							//	�R�}���h�I��
			
			else if ( scom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// �����~
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
			}
			else if ( ( scom[us_rp] <=  GO71 ) && ( scom[us_rp] >=  GO1) )
			{
				if( scom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, 0 );						// �������s�R�}���h�A����ԑO�i�i�ŏI���x�Ȃ��j
				}
				else{
					
					/* �ǂ̐؂�ڕ␳ */
					if( ( scom[us_rp+1] == R90S )   || ( scom[us_rp+1] == L90S ) ){
						bl_isWallCut = MAP_setWallCut( scom[us_rp+1] );		// �R�[�i�[�O�ɕǂ���������ǂ̐؂�ڕ␳���s���ݒ������
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, MOT_getSuraStaSpeed() );		// �������s�R�}���h�A����ԑO�i�i�ŏI���x����j
				}
			}
			else if( scom[us_rp] == R90S )
			{
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
			}
			else if( scom[us_rp] == L90S )
			{
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
			}
			us_rp++;
			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}
			
		}
	}
	/* �΂߃��[�h */
	else if( en_driveType == MAP_DRIVE_SKEW )
	{
		while(1)
		{
			MAP_refPos( tcom[us_rp] );									// ���s�����R�}���h���I�������ʒu�ɍX�V
			
			if ( tcom[us_rp] == CEND  ) break;							//	�R�}���h�I��

			else if ( tcom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// �����~
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
			}
			else if ( ( tcom[us_rp] <=  GO71 ) && ( tcom[us_rp] >=  GO1) )
			{
				if( tcom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, 0 );						// �������s�R�}���h�A����ԑO�i�i�ŏI���x�Ȃ��j
				}
				else{
					
					/* �ǂ̐؂�ڕ␳ */
					if( ( tcom[us_rp+1] == R90S )   || ( tcom[us_rp+1] == L90S )   || 
					 	( tcom[us_rp+1] == RS135N ) || ( tcom[us_rp+1] == LS135N ) 
					 ){
						bl_isWallCut = MAP_setWallCut( tcom[us_rp+1] );		// �R�[�i�[�O�ɕǂ���������ǂ̐؂�ڕ␳���s���ݒ������
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, MOT_getSuraStaSpeed() );		// �������s�R�}���h�A����ԑO�i�i�ŏI���x����j
				}
			}
			else if ( ( tcom[us_rp] <=  NGO71 ) && ( tcom[us_rp] >=  NGO1) )
			{
				MOT_goSkewBlock_FinSpeed( (FLOAT)(tcom[us_rp]-81)*0.5f, MOT_getSuraStaSpeed());	// �΂ߒ������s�R�}���h�A����ԑO�i�i�ŏI���x����j
			}
			else
			{
				switch( tcom[us_rp] )
				{

					/* ���i �� ���i */
					case R90S:		MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			break;
					case L90S:		MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			break;
					
					/* ���i �� �΂� */
					case RS45N:		MOT_goSla( MOT_R45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case LS45N:		MOT_goSla( MOT_L45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case RS135N:	MOT_goSla( MOT_R135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;
					case LS135N:	MOT_goSla( MOT_L135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;

					/* �΂� �� ���i */
					case RN45S:		MOT_goSla( MOT_R45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case LN45S:		MOT_goSla( MOT_L45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case RN135S:	MOT_goSla( MOT_R135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;
					case LN135S:	MOT_goSla( MOT_L135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;

					/* �΂� �� �΂� */
					case RN90N:		MOT_goSla( MOT_R90S_N, PARAM_getSra( SLA_N90 ) ); 		break;
					case LN90N:		MOT_goSla( MOT_L90S_N, PARAM_getSra( SLA_N90 ) );		break;
				}
			}
			us_rp++;
			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// �u���[�L
				DCM_brakeMot( DCM_L );		// �u���[�L
				break;
			}
		}
	}

}

// *************************************************************************
//   �@�\		�F �I�[�g�X�^�[�g�Z�b�g
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
/*PUBLIC void MAP_autostart()
{
	enMAP_HEAD_DIR	en_headset;
	
	// ���H�T�� 
//	POS_clr();			// debug
//	POS_sta();			// debug
	MAP_setPos( 0, 0, NORTH );							// �X�^�[�g�ʒu
	MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA );			// �S�[���ݒ�
//	POS_stop();			// debug
			
	// �A��̃X�����[���T�� 
	TIME_wait(1000);
	LED4 = LED4_ALL_OFF;
	MAP_searchGoal( 0, 0, SEARCH, SEARCH_SURA );
	TIME_wait(1000);
			
	if ( SW_ON == SW_INC_PIN ){}
	else{
		//�ŒZ���s1���
//		PARAM_setCntType( TRUE );								// �ŒZ���s
		MAP_setPos( 0, 0, NORTH );								// �X�^�[�g�ʒu
		MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
		MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// �h���C�u�R�}���h�쐬
		MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
//		MAP_makeSkewCmdList();
			
		MOT_goHitBackWall();
		MOT_goBlock_FinSpeed( 0.3, 0 );
		TIME_wait(1000);
			
		MOT_setTrgtSpeed(SEARCH_SPEED*2);
		MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// �X�����[���J�n���x�ݒ�
		PARAM_setSpeedType( PARAM_ST,   PARAM_NORMAL );							// [���i] ���x����
		PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [����] ���x����
		PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [�X��] ���x����
			
		MAP_drive( MAP_DRIVE_SURA );
		TIME_wait(500);
		MOT_turn(MOT_R180);
		MAP_actGoalLED();
		
		//���H
		en_headset = (enMAP_HEAD_DIR)( (&en_endDir + 2) & (MAP_HEAD_DIR_MAX-1) );
		
//		PARAM_setCntType( TRUE );								// �ŒZ���s
		MAP_setPos( GOAL_MAP_X, GOAL_MAP_Y, en_headset );								// �X�^�[�g�ʒu
		MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
		MAP_makeCmdList( GOAL_MAP_X, GOAL_MAP_Y, en_headset, 0, 0, &en_endDir );		// �h���C�u�R�}���h�쐬
		MAP_makeSuraCmdList();													// �X�����[���R�}���h�쐬
//		MAP_makeSkewCmdList();
			
		MOT_goHitBackWall();
		MOT_goBlock_FinSpeed( 0.3, 0 );
		TIME_wait(1000);
		
		//�ŒZ���s2���

}
*/