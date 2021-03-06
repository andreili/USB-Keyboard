#ifndef _KBD_MATRIX_DATA_H_
#define _KBD_MATRIX_DATA_H_

#include <inttypes.h>
#include "kbd_matrix.h"
#include "kbd_global.h"

#define KMX(row, col) ((row << 4) | (col & 0x0f))

#include "kbd_mtx_rk86.h"

																		/* 0x00				0x01				0x02				0x03				0x04				0x05				0x06				0x07 */
const uint8_t kbd_mc7007_lat[128] ={KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX( 3, 5),	KMX( 3, 9),	KMX( 2, 3),	KMX( 2,10), /* 0x07 */
																		KMX( 1, 6),	KMX( 2, 2),	KMX( 2, 8),	KMX( 6, 9),	KMX( 3, 6),	KMX( 1, 2),	KMX( 2, 5),	KMX( 2, 9), /* 0x0f */
																		KMX( 4, 5),	KMX( 2, 7),	KMX( 3, 8),	KMX( 2, 6),	KMX( 3, 2),	KMX( 3, 7),	KMX( 4, 4),	KMX( 4, 7), /* 0x17 */
																		KMX( 2, 4),	KMX( 5,10),	KMX( 3, 4),	KMX( 6, 9),	KMX( 3, 3),	KMX( 6,10),	KMX( 1, 3),	KMX( 1, 4), /* 0x1f */
																		KMX( 1, 5),	KMX( 0, 6),	KMX( 1, 7),	KMX( 1, 8),	KMX( 0, 9),	KMX( 0,10),	KMX( 7,10),	KMX( 7, 9), /* 0x27 */
																		KMX( 6, 6),	KMX( 0, 1),	KMX( 5, 5),	KMX( 1, 1),	KMX( 4, 6),	KMX( 7, 8),	KMX( 0, 2),	KMX( 1, 9), /* 0x2f */
																		KMX( 1,10),	KMX( 5, 9),	KMX(15,15),	KMX( 7, 6),	KMX( 4, 3),	KMX(15,15),	KMX( 4,10),	KMX( 5, 8), /* 0x37 */
																		KMX( 7, 7),	KMX( 2, 1),	KMX( 0, 3),	KMX( 0, 4),	KMX( 0, 5),	KMX( 0, 7),	KMX( 0, 8),	KMX( 6, 4), /* 0x3f */
																		KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15), /* 0x47 */
																		KMX(15,15),	KMX( 7, 4),	KMX( 6, 5),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX( 5, 6),	/* 0x4f */
																		KMX( 4, 9),	KMX( 5, 7),	KMX( 6, 7),	KMX(15,15),	KMX(15,15),	KMX( 0, 0),	KMX( 7, 8),	KMX( 0, 2), /* 0x57 */
																		KMX( 6, 6),	KMX( 5, 2),	KMX( 6, 2),	KMX( 7, 2),	KMX( 5, 3),	KMX( 6, 3),	KMX( 7, 3),	KMX( 5, 0), /* 0x5f */
																		KMX( 6, 0),	KMX( 7, 0),	KMX( 5, 1),	KMX( 6, 1),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15), /* 0x67 */
																		KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15), /* 0x6f */
																		KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15), /* 0x77 */
																		KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15),	KMX(15,15)};/* 0x7f */
const uint8_t kbd_mc7007_f[8] =		 {KMX( 4, 1), KMX( 4, 0), KMX(15,15), KMX(15,15), KMX( 4, 1), KMX( 4, 0), KMX(15,15), KMX(15,15)};

kbd_matrix_t kbd_mc7007 = {kbd_mc7007_lat, 0, kbd_mc7007_f};

#endif
