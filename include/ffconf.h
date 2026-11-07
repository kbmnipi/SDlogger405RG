/*---------------------------------------------------------------------------/
/  FatFs - FAT file system module configuration file   (R0.12c / ST port)    /
/----------------------------------------------------------------------------/
/  Project : SDlogger405RG (STM32F405, HAL_SD backend)                       /
/---------------------------------------------------------------------------*/

#ifndef _FFCONF
#define _FFCONF	68300	/* Revision ID (must equal the one in ff.h) */

/*---------------------------------------------------------------------------/
/ Function Configurations                                                   /
/---------------------------------------------------------------------------*/
#define _FS_READONLY	0	/* 0: Read/Write */
#define _FS_MINIMIZE	0	/* Keep all basic API functions */
#define _USE_STRFUNC	0	/* Disable string I/O */
#define _USE_FIND		0	/* Disable filtered directory read */
#define _USE_MKFS		0	/* 0: No formatting support */
#define _USE_FASTSEEK	0	/* Disable fast-seek linkmap */
#define _USE_EXPAND		0	/* Disable f_expand() */
#define _USE_CHMOD		0	/* Disable attribute manipulation */
#define _USE_LABEL		1	/* Enable volume label functions */
#define _USE_FORWARD	0	/* Disable f_forward() */

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations                                       /
/---------------------------------------------------------------------------*/
#define _CODE_PAGE	437	/* OEM code page (US) */
#define _USE_LFN		0	/* 0: Short file names only */
#define _MAX_LFN	255	/* Max LFN length */
#define _LFN_UNICODE	0	/* API strings are ANSI/OEM */
#define _STRF_ENCODE	3	/* Inert */
#define _FS_RPATH	0	/* Disable relative-path support */

/*---------------------------------------------------------------------------/
/ Drive / Volume Configurations                                             /
/---------------------------------------------------------------------------*/
#define _VOLUMES	1	/* One logical drive */
#define _STR_VOLUME_ID	0	/* No string volume IDs */
#define _MULTI_PARTITION	0	/* Single partition */
#define _MIN_SS		512	/* Supported sector size */
#define _MAX_SS		512	/* Max sector size */
#define _USE_TRIM	0	/* No ATA-TRIM support */
#define _FS_NOFSINFO	0	/* Trust FAT32 FSINFO */
#define _FS_TIMEOUT	1	/* Enable disk-access timeout */

/*---------------------------------------------------------------------------/
/ System Configurations                                                     /
/---------------------------------------------------------------------------*/
#define _FS_TINY	0	/* Normal buffer config */
#define _FS_EXFAT	0	/* exFAT disabled */
#define _FS_NORTC	1	/* No RTC: use fixed timestamps */
#define _NORTC_MON	1	/* Fixed month */
#define _NORTC_MDAY	1	/* Fixed day */
#define _NORTC_YEAR	2026	/* Fixed year */
#define _FS_LOCK	0	/* Disable file-lock control */
#define _FS_REENTRANT	0	/* No RTOS */
#define _USE_MUTEX	0	/* Inert */

#endif /* _FFCONF */