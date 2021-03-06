#ifndef _KBD_RK86_DATA_H_
#define _KBD_RK86_DATA_H_

#include <inttypes.h>

const uint8_t kbd_rk86_lat[128] = {
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(4,1),KMX(4,2),KMX(4,3),KMX(4,4),
						KMX(4,5),KMX(4,6),KMX(4,7),KMX(5,0),KMX(5,1),KMX(5,2),KMX(5,3),KMX(5,4),
						KMX(5,5),KMX(5,6),KMX(5,7),KMX(6,0),KMX(6,1),KMX(6,2),KMX(6,3),KMX(6,4),
						KMX(6,5),KMX(6,6),KMX(6,7),KMX(7,0),KMX(7,1),KMX(7,2),KMX(2,1),KMX(2,2),
						KMX(2,3),KMX(2,4),KMX(2,5),KMX(2,6),KMX(2,7),KMX(3,0),KMX(3,1),KMX(2,0),
						KMX(1,2),KMX(0,2),KMX(1,3),KMX(1,0),KMX(7,7),KMX(3,5),KMX(3,3),KMX(7,3),
						KMX(7,5),KMX(7,4),KMX(15,15),KMX(3,2),KMX(7,6),KMX(4,0),KMX(3,4),KMX(3,6),
						KMX(3,7),KMX(15,15),KMX(0,3),KMX(0,4),KMX(0,5),KMX(0,6),KMX(0,7),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(0,1),KMX(0,0),KMX(15,15),KMX(15,15),KMX(1,1),KMX(15,15),KMX(1,6),
						KMX(1,4),KMX(1,7),KMX(1,5),KMX(15,15),KMX(3,7),KMX(3,2),KMX(3,5),KMX(3,3),
						KMX(1,2),KMX(2,1),KMX(2,2),KMX(2,3),KMX(2,4),KMX(2,5),KMX(2,6),KMX(2,7),
						KMX(3,0),KMX(3,1),KMX(2,0),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15)};
const uint8_t kbd_rk86_rus[128] = {
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),
						KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15)};
const uint8_t kbd_rk86_f[8] = {KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15),KMX(15,15)};
kbd_matrix_t kbd_rk86 = {kbd_rk86_lat, kbd_rk86_rus, kbd_rk86_f};

#endif
