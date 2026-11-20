#include "bsp.h"

/*
 * 主机测试用的按键替身。
 *
 * 测试用例通过 host_key_press()/host_key_release() 脚本化按键，
 * BallTest_AppTask() 仍然调用真实的 user_key_take_edge()/is_pressed() 逻辑接口。
 */

static bool s_pressed[USER_KEY_COUNT];
static bool s_edge[USER_KEY_COUNT];

void host_key_reset(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)USER_KEY_COUNT; i++) {
        s_pressed[i] = false;
        s_edge[i] = false;
    }
}

void host_key_press(user_key_id_t key)
{
    if (key >= USER_KEY_COUNT) {
        return;
    }
    s_pressed[key] = true;
    s_edge[key] = true;
}

void host_key_release(user_key_id_t key)
{
    if (key >= USER_KEY_COUNT) {
        return;
    }
    s_pressed[key] = false;
}

void user_key_init(void)
{
    host_key_reset();
}

bool user_key_is_pressed(user_key_id_t key)
{
    if (key >= USER_KEY_COUNT) {
        return false;
    }
    return s_pressed[key];
}

bool user_key_take_edge(user_key_id_t key)
{
    bool edge;

    if (key >= USER_KEY_COUNT) {
        return false;
    }

    edge = s_edge[key];
    s_edge[key] = false;
    return edge;
}
