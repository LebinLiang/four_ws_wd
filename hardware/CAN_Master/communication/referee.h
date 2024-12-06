#ifndef REFEREE_H
#define REFEREE_H

//裁判信息解读与透传数据帧封装程序
//#include "bsp_remote_control.h"
#include "stdint.h"

 uint16_t referee_data_solve(uint8_t *frame);

 void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len);

void referee_unpack_fifo_data(void);

#endif
