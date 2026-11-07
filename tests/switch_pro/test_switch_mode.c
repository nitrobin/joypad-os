// Host-side protocol test: compile the real Switch mode with TinyUSB stubs.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define USBD_MODE_H
#define USBD_H
#define USB_OUTPUT_MODE_SWITCH 5
#include "tusb.h"

typedef struct {
    int16_t accel[3], gyro[3];
    uint16_t accel_range, gyro_range;
} input_event_t;
typedef struct {
    int16_t accel[3], gyro[3];
    uint8_t left_x, left_y, right_x, right_y, l2_analog, r2_analog;
    bool has_motion;
} profile_output_t;
typedef struct {
    uint8_t rumble_left, rumble_right, led_player, led_r, led_g, led_b;
    bool dirty;
} output_feedback_t;
typedef struct {
    const char *name;
    int mode;
    const uint8_t *(*get_device_descriptor)(void);
    const uint8_t *(*get_config_descriptor)(void);
    const uint8_t *(*get_report_descriptor)(void);
    void (*init)(void);
    bool (*send_report)(uint8_t, const input_event_t *, const profile_output_t *, uint32_t);
    bool (*is_ready)(void);
    void (*handle_output)(uint8_t, const uint8_t *, uint16_t);
    bool (*get_feedback)(output_feedback_t *);
    uint16_t (*get_report)(uint8_t, hid_report_type_t, uint8_t *, uint16_t);
    void (*task)(void);
} usbd_mode_t;
static bool usbd_l2_digital(const profile_output_t *p, uint32_t b)
{ return (b & (1u << 6)) || p->l2_analog >= 5; }
static bool usbd_r2_digital(const profile_output_t *p, uint32_t b)
{ return (b & (1u << 7)) || p->r2_analog >= 5; }

static uint8_t sent_id, sent_data[63];
static uint32_t fake_ms;
bool tud_hid_ready(void) { return true; }
bool tud_hid_report(uint8_t id, const void *data, uint16_t len)
{
    assert(len == 63);
    sent_id = id;
    memcpy(sent_data, data, 63);
    return true;
}
uint32_t platform_time_ms(void) { return fake_ms; }
void platform_get_unique_id(uint8_t *dst, size_t len)
{ for (size_t i = 0; i < len; ++i) dst[i] = (uint8_t)(i + 1); }

#include "../../src/usb/usbd/modes/switch_mode.c"

static int16_t le16(const uint8_t *p)
{ return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

int main(void)
{
    switch_mode.init();
    assert(controller_addr[0] == 2 && controller_addr[1] == 4);
    input_event_t e = { .accel_range = 4000, .gyro_range = 2000 };
    profile_output_t p = {
        .left_x = 0, .left_y = 255, .right_x = 255, .right_y = 0,
        .l2_analog = 100, .has_motion = true,
        .accel = {8192, -8192, 0}, .gyro = {1024, -1024, 0},
    };
    uint32_t buttons = JP_BUTTON_B1 | JP_BUTTON_B2 | JP_BUTTON_DU | JP_BUTTON_A1;
    assert(switch_mode.send_report(0, &e, &p, buttons));
    assert(sent_id == 0x30);
    assert((sent_data[2] & 0x0c) == 0x0c); // B + A
    assert((sent_data[3] & 0x10) != 0); // HOME
    assert((sent_data[4] & 0x82) == 0x82); // UP + ZL
    assert(le16(sent_data + 12) == 0); // IMU stays off until enabled

    // Switch asks for IMU, then receives three repeated 12-byte samples.
    uint8_t imu_on[11] = {0};
    imu_on[9] = 0x40; imu_on[10] = 1;
    switch_mode.handle_output(0x01, imu_on, sizeof(imu_on));
    switch_mode.task();
    assert(sent_id == 0x21 && sent_data[13] == 0x40);
    assert(switch_mode.send_report(0, &e, &p, buttons));
    for (int base = 12; base <= 36; base += 12) {
        assert(le16(sent_data + base) == 4096);
        assert(le16(sent_data + base + 2) == -4096);
        assert(le16(sent_data + base + 6) == 1024);
    }
    // 48-byte SPI requests must not overrun the 64-byte HID reply.
    uint8_t spi[16] = {0};
    spi[9] = 0x10; spi[10] = 0x20; spi[11] = 0x60; spi[14] = 48;
    switch_mode.handle_output(0x01, spi, sizeof(spi));
    switch_mode.task();
    assert(sent_id == 0x21 && sent_data[12] == 0x90);
    assert(sent_data[13] == 0x10);
    assert(sent_data[18] == 0x30); // request length clamped to 48
    uint8_t get[63] = {0};
    assert(switch_mode.get_report(0x30, HID_REPORT_TYPE_INPUT, get, sizeof(get)) == 63);
    assert(get[2] == report[3]);
    return 0;
}
