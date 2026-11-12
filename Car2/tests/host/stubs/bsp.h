#ifndef BSP_H
#define BSP_H

/*
 * 主机端 BSP 聚合头替身。
 *
 * 仅用于对 User/App/ball_test.c 做语法/类型检查，不参与 Keil 编译，
 * 也不链接任何真实硬件实现。真实工程的 bsp.h 会引入全部 BSP 模块。
 */
#include "bsp_common.h"
#include "SystemTime/system_time.h"

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

#define LCD_W           (280U)
#define LCD_H           (240U)

#define WHITE           (0xFFFFU)
#define BLACK           (0x0000U)
#define RED             (0xF800U)
#define GREEN           (0x07E0U)
#define CYAN            (0x7FFFU)
#define YELLOW          (0xFFE0U)
#define GRAY            (0x8430U)
#define LGRAY           (0xC618U)

void LCD_Init(void);
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);

typedef enum {
    USER_KEY_K1 = 0,
    USER_KEY_K2 = 1,
    USER_KEY_COUNT
} user_key_id_t;

void user_key_init(void);
bool user_key_is_pressed(user_key_id_t key);
bool user_key_take_edge(user_key_id_t key);

#endif
