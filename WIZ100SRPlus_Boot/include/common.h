
#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>

#define MAJOR_VER       101
#define MINOR_VER       0
#define MAINTENANCE_VER 0

#define SOCK_DATA       0
#define SOCK_TFTP       1
#define SOCK_CONFIG     2
#define SOCK_DHCP       3
#define SOCK_DNS        4

#define OP_COMMAND      0
#define OP_DATA         1

#define APP_BASE        0x00008000          // Boot Size 32K
#define WORK_BUF_SIZE   2048

#define DEVICE_BOOT_ADDR					0x00000000
#define DEVICE_BOOT_SIZE					(32*1024)
#define DEVICE_APP_MAIN_ADDR				(DEVICE_BOOT_ADDR + DEVICE_BOOT_SIZE)

extern uint8_t op_mode;

#endif
