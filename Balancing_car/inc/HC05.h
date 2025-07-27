#ifndef __HC05_H
#define __HC05_H


#include "allheader.h"

#define HC05_PORT 2

#define 	run_car     'F'
#define 	back_car    'B'
#define 	left_car    'L'
#define 	right_car   'R'
#define 	stop_car    'S'


extern volatile char neworder;
extern volatile char order;

void HC05_init(uint8_t hc05_port);
void Check_Bluetooth_Order();





#endif


