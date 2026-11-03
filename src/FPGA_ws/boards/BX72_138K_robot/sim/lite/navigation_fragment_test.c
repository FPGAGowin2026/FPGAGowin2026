/* Exercise the production task assembler after the RTL has validated each packet. */
#include "navigation.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t bx72_nav_test_mmio[0x2b0u / 4u];
static uint8_t payload[72];

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static void task_payload(uint32_t id, uint32_t sent_us, uint32_t expiry_us)
{
    memset(payload, 0, sizeof(payload));
    payload[0] = 1u; /* version */
    payload[1] = 1u; /* enqueue */
    payload[2] = 3u; /* FPGA standalone */
    payload[3] = 2u; /* position only */
    put32(payload + 4, id);
    put32(payload + 8, 1u); /* odom domain */
    put32(payload + 12, 7u); /* boot */
    put32(payload + 16, 65536u); /* x = +1 m, Q16 */
    put32(payload + 20, (uint32_t)-32768); /* y = -0.5 m, Q16 */
    put16(payload + 32, 120u);
    put16(payload + 34, 300u);
    put16(payload + 36, 40u);
    put16(payload + 38, 100u);
    put32(payload + 44, expiry_us);
    put32(payload + 60, sent_us);
}

static void fragment(uint8_t index, uint32_t sequence, uint32_t now_us)
{
    uint8_t frame[32] = {0};
    uint32_t words[8];
    unsigned i;
    frame[0] = 'B'; frame[1] = 'X'; frame[2] = '7'; frame[3] = '2';
    frame[4] = 1u; frame[5] = 8u; frame[7] = 32u; /* validated type-8 form */
    put32(frame + 8, sequence);
    put32(frame + 16, 2u); /* session token */
    frame[20] = index;
    memcpy(frame + 21, payload + (unsigned)index * 9u, 9u);
    for (i = 0; i < 8; ++i) words[i] = read_u32_be(frame + i * 4u);
    bx72_nav_fragment(words, now_us);
}

static void complete(uint32_t sequence, uint32_t now_us)
{
    unsigned i;
    for (i = 0; i < 8; ++i) fragment((uint8_t)i, sequence + i, now_us + i * 1000u);
}

int main(void)
{
    memset(bx72_nav_test_mmio, 0, sizeof(bx72_nav_test_mmio));
    bx72_nav_test_mmio[BX72_IO_BOOT_ID / 4u] = 7u;
    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN / 4u] = 1u;
    bx72_nav_test_mmio[BX72_IO_SESSION_LO / 4u] = 2u;
    bx72_nav_init();
    bx72_nav_sync_inputs(1000000u, 1u, 0u);

    task_payload(1u, 1000000u, 12000000u);
    complete(100u, 1000000u);
    assert(task_count == 1u && task_queue[task_head].task_id == 1u);
    assert(task_queue[task_head].goal_x_mm_q16 == 1000ll * NAV_Q16_ONE);
    assert(task_queue[task_head].goal_y_mm_q16 == -500ll * NAV_Q16_ONE);
    bx72_nav_sync_inputs(1009000u, 2u, 0u); /* zero-speed takeover */
    assert(task_count == 0u && event_status == BX72_NAV_STATUS_CANCELLED);

    task_payload(2u, 1010000u, 12000000u);
    fragment(0u, 200u, 1010000u);
    assert(fragment_collecting);
    bx72_nav_sync_inputs(1011000u, 2u, 1u); /* mode transition during assembly */
    fragment(1u, 201u, 1012000u);
    assert(!fragment_collecting && task_count == 0u && event_status == BX72_NAV_STATUS_REJECTED);
    fragment(0u, 300u, 1013000u);
    fragment(2u, 301u, 1014000u);
    assert(!fragment_collecting && task_count == 0u);
    fragment(0u, 400u, 1020000u);
    fragment(1u, 401u, 1220001u);
    assert(!fragment_collecting && task_count == 0u); /* 200 ms fragment deadline */

    task_payload(3u, 1300000u, 1300001u);
    complete(500u, 1300000u);
    assert(task_count == 0u && event_status == BX72_NAV_STATUS_REJECTED);
    task_payload(3u, 1400000u, 2400000u);
    complete(0xfffffffcu, 1400000u); /* fragment sequence wraps */
    assert(task_count == 1u && task_queue[task_head].task_id == 3u);
    bx72_nav_sync_inputs(1408000u, 3u, 1u);
    assert(task_count == 0u && event_status == BX72_NAV_STATUS_CANCELLED);

    task_payload(4u, 1500000u, 2500000u);
    fragment(0u, 600u, 1500000u);
    bx72_nav_task_input_lost(1501000u);
    fragment(1u, 601u, 1502000u);
    assert(!fragment_collecting && task_count == 0u);
    puts("PASS eight task fragments, mode/stop handoff, order, timeout, wrap and overflow");
    return 0;
}
