// Nintendo Switch Pro Controller USB mode.
// SPDX-License-Identifier: Apache-2.0
// Protocol layout and calibration responses adapted from Pico-NS2-Waker (MIT):
// https://github.com/david419kr/Pico-NS2-Waker
#include "tusb.h"
#include "../usbd_mode.h"
#include "../usbd.h"
#include "descriptors/switch_descriptors.h"
#include "core/buttons.h"
#include "platform/platform.h"
#include <string.h>

#define SW_REPORT_SIZE 64
#define SW_FULL_REPORT 0x30
#define SW_SUB_REPLY 0x21
#define SW_USB_REPLY 0x81

static uint8_t report[SW_REPORT_SIZE];
static uint8_t reply[SW_REPORT_SIZE];
static uint8_t queued_replies[3][SW_REPORT_SIZE];
static uint8_t queued_count;
static uint8_t timer_byte;
static uint8_t player_lights;
static uint8_t report_mode;
static bool imu_enabled;
static bool vibration_enabled;
static output_feedback_t feedback;
static bool feedback_pending;
static uint32_t last_report_ms;
// Stable, locally administered identity derived from the board ID.
static uint8_t controller_addr[6];

static void put16(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)((uint16_t)value >> 8);
}

static void pack_stick(uint8_t *dst, uint16_t x, uint16_t y)
{
    dst[0] = (uint8_t)x;
    dst[1] = (uint8_t)(((x >> 8) & 15u) | ((y & 15u) << 4));
    dst[2] = (uint8_t)(y >> 4);
}

static uint16_t axis12(uint8_t value)
{
    return (uint16_t)(((uint32_t)value * 4095u + 127u) / 255u);
}

static int16_t clamp16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static uint8_t rumble_level(const uint8_t *data)
{
    // Switch HD-rumble packet: high-band amplitude is in bits 1..7 of byte 1,
    // low-band amplitude is in byte 3. Neutral packets mean silence.
    if ((!data[0] && !data[1] && !data[2] && !data[3]) ||
        (!data[0] && data[1] == 1 && data[2] == 0x40 && data[3] == 0x40)) return 0;
    uint32_t high = (uint32_t)(data[1] & 0xfeu) * 78u / 254u;
    uint32_t low = (uint32_t)data[3] * 22u / 255u;
    return (uint8_t)((high + low) * 255u / 100u);
}

static void sample_rumble(const uint8_t *data, uint16_t len)
{
    if (len < 10) return;
    uint8_t left = rumble_level(data + 2);
    uint8_t right = rumble_level(data + 6);
    if (feedback.rumble_left != left || feedback.rumble_right != right) {
        feedback.rumble_left = left;
        feedback.rumble_right = right;
        feedback_pending = true;
    }
}

static void copy_window(uint8_t *dst, uint8_t len, uint32_t address,
                        uint32_t base, const uint8_t *src, uint8_t src_len)
{
    for (uint8_t i = 0; i < len; ++i) {
        uint32_t pos = address + i;
        if (pos >= base && pos - base < src_len) dst[i] = src[pos - base];
    }
}

static void spi_read(uint8_t *dst, const uint8_t *args, uint16_t args_len)
{
    if (args_len < 5) return;
    uint32_t address = (uint32_t)args[0] | ((uint32_t)args[1] << 8) |
                       ((uint32_t)args[2] << 16) | ((uint32_t)args[3] << 24);
    uint8_t len = args[4] > 0x30 ? 0x30 : args[4];
    memcpy(dst, args, 4);
    dst[4] = len;
    memset(dst + 5, 0xff, len);
    static const uint8_t serial[16] = {
        '0','0','0','0','0','0','0','0','0','0','0','1',0xff,0xff,0xff,0xff};
    static const uint8_t type[2] = {0x03, 0xa0};
    static const uint8_t color_valid = 1;
    static const uint8_t colors[12] = {
        0x32,0x32,0x32,0xff,0xff,0xff,0x32,0x32,0x32,0x32,0x32,0x32};
    uint8_t imu_cal[24] = {0};
    for (int i = 0; i < 3; ++i) {
        put16(imu_cal + 6 + i * 2, 0x4000);
        put16(imu_cal + 18 + i * 2, 0x343b);
    }
    uint8_t left_cal[9], right_cal[9];
    pack_stick(left_cal, 2047, 2047);
    pack_stick(left_cal + 3, 2048, 2048);
    pack_stick(left_cal + 6, 2048, 2048);
    pack_stick(right_cal, 2048, 2048);
    pack_stick(right_cal + 3, 2048, 2048);
    pack_stick(right_cal + 6, 2047, 2047);
    uint8_t horizontal[6];
    put16(horizontal, -688);
    put16(horizontal + 2, 0);
    put16(horizontal + 4, 4038);
    static const uint16_t stick_values[12] = {
        0x0019, 0x04cd, 0x00ae, 0x0e14,
        0x02ee, 0x02ee, 0x02ee, 0x02ee,
        0x0ab4, 0x0ab4, 0x0496, 0x0496,
    };
    uint8_t stick_params[18];
    for (unsigned i = 0; i < 6; ++i) {
        pack_stick(stick_params + i * 3, stick_values[i * 2], stick_values[i * 2 + 1]);
    }
    copy_window(dst + 5, len, address, 0x6000, serial, sizeof(serial));
    copy_window(dst + 5, len, address, 0x6012, type, sizeof(type));
    copy_window(dst + 5, len, address, 0x601b, &color_valid, 1);
    copy_window(dst + 5, len, address, 0x6020, imu_cal, sizeof(imu_cal));
    copy_window(dst + 5, len, address, 0x603d, left_cal, sizeof(left_cal));
    copy_window(dst + 5, len, address, 0x6046, right_cal, sizeof(right_cal));
    copy_window(dst + 5, len, address, 0x6050, colors, sizeof(colors));
    copy_window(dst + 5, len, address, 0x6080, horizontal, sizeof(horizontal));
    copy_window(dst + 5, len, address, 0x6086, stick_params, sizeof(stick_params));
    copy_window(dst + 5, len, address, 0x6098, stick_params, sizeof(stick_params));
}

static void make_full_report(void)
{
    report[0] = SW_FULL_REPORT;
    report[1] = timer_byte++;
    report[2] = 0x81;
    report[12] = vibration_enabled ? 0x38 : 0x30;
}

static void make_reply_base(uint8_t id)
{
    memcpy(reply, report, sizeof(reply));
    reply[0] = id;
    reply[1] = timer_byte++;
}

static void queue_reply(void)
{
    if (queued_count < 3) memcpy(queued_replies[queued_count++], reply, sizeof(reply));
}

static void respond_usb(uint8_t command)
{
    if (command == 0x04) return; // no-timeout command has no reply
    memset(reply, 0, sizeof(reply));
    reply[0] = SW_USB_REPLY;
    reply[1] = command;
    reply[3] = 0x03;
    if (command == 0x01) {
        for (unsigned i = 0; i < 6; ++i) reply[4 + i] = controller_addr[5 - i];
    }
    if (command == 0x06) report_mode = 0;
}

static void respond_subcommand(uint8_t command, const uint8_t *args, uint16_t len)
{
    make_reply_base(SW_SUB_REPLY);
    memset(reply + 13, 0, SW_REPORT_SIZE - 13);
    reply[13] = 0x80;
    reply[14] = command;
    uint8_t *payload = reply + 15;
    switch (command) {
    case 0x01: // manual pairing, wired identity only
        reply[13] = 0x81;
        if (len && args[0] == 1) {
            static const char name[] = "Pro Controller";
            payload[0] = 1;
            for (unsigned i = 0; i < 6; ++i) payload[1 + i] = controller_addr[5 - i];
            payload[8] = 0x25; payload[9] = 0x08;
            memcpy(payload + 10, name, sizeof(name) - 1);
        } else if (len && args[0] == 2) {
            static const uint8_t key[16] = {
                0xeb,0xe0,0xf9,0xf6,0xcc,0xda,0xd0,0x2e,
                0x25,0x30,0x0f,0x1f,0x6f,0x7f,0x4c,0x5d};
            payload[0] = 2;
            memcpy(payload + 1, key, sizeof(key));
        } else if (len) payload[0] = args[0] == 4 ? 3 : args[0];
        break;
    case 0x02: // device info
        reply[13] = 0x82;
        payload[0] = 0x03; payload[1] = 0x48;
        payload[2] = 0x03; payload[3] = 0x02;
        memcpy(payload + 4, controller_addr, 6);
        payload[10] = 1; payload[11] = 3;
        break;
    case 0x03: // input report mode
        if (len) report_mode = args[0];
        break;
    case 0x10: // SPI calibration and controller metadata
        reply[13] = 0x90;
        {
            uint8_t spi_payload[54] = {0};
            spi_read(spi_payload, args, len);
            memcpy(payload, spi_payload, sizeof(reply) - 15);
        }
        break;
    case 0x11: case 0x12: // SPI write/erase acknowledgement
        payload[0] = 0;
        break;
    case 0x30: // player lights
        if (len) {
            player_lights = args[0];
            feedback.led_player = (uint8_t)((player_lights & 0x0f) ?
                (__builtin_ctz((unsigned)(player_lights & 0x0f)) + 1) : 0);
            feedback_pending = true;
        }
        break;
    case 0x31: // get player lights
        reply[13] = 0xb0;
        payload[0] = player_lights;
        break;
    case 0x33: payload[0] = 0x03; break;
    case 0x40: if (len) imu_enabled = args[0] != 0; break;
    case 0x41: case 0x42: break; // IMU sensitivity/register writes
    case 0x43: // IMU register read
        reply[13] = 0xc0;
        payload[0] = len ? args[0] : 0;
        payload[1] = len > 1 ? args[1] : 1;
        if (payload[1] > 0x20) payload[1] = 0x20;
        break;
    case 0x48: if (len) vibration_enabled = args[0] != 0; break;
    case 0x50:
        reply[13] = 0xd0;
        payload[0] = 0x40; payload[1] = 0x06;
        break;
    default: break;
    }
}

static void switch_mode_init(void)
{
    uint8_t board_id[8] = {0};
    platform_get_unique_id(board_id, sizeof(board_id));
    controller_addr[0] = 0x02;
    memcpy(controller_addr + 1, board_id + 3, 5);
    memset(report, 0, sizeof(report));
    memset(reply, 0, sizeof(reply));
    memset(&feedback, 0, sizeof(feedback));
    queued_count = 0;
    feedback_pending = false;
    timer_byte = player_lights = report_mode = 0;
    imu_enabled = vibration_enabled = false;
    last_report_ms = 0;
    pack_stick(report + 6, 2048, 2048);
    pack_stick(report + 9, 2048, 2048);
    make_full_report();
}

static bool switch_mode_is_ready(void)
{
    return tud_hid_ready() && queued_count == 0;
}

static bool switch_mode_send_report(uint8_t player_index,
                                    const input_event_t *event,
                                    const profile_output_t *out,
                                    uint32_t buttons)
{
    (void)player_index;
    make_full_report();
    report[3] = report[4] = report[5] = 0;
    // Physical face positions: Cross/B, Circle/A, Square/Y, Triangle/X.
    if (buttons & JP_BUTTON_B1) report[3] |= 0x04;
    if (buttons & JP_BUTTON_B2) report[3] |= 0x08;
    if (buttons & JP_BUTTON_B3) report[3] |= 0x01;
    if (buttons & JP_BUTTON_B4) report[3] |= 0x02;
    if (buttons & JP_BUTTON_R1) report[3] |= 0x40;
    if (usbd_r2_digital(out, buttons)) report[3] |= 0x80;
    if (buttons & JP_BUTTON_S1) report[4] |= 0x01;
    if (buttons & JP_BUTTON_S2) report[4] |= 0x02;
    if (buttons & JP_BUTTON_R3) report[4] |= 0x04;
    if (buttons & JP_BUTTON_L3) report[4] |= 0x08;
    if (buttons & JP_BUTTON_A1) report[4] |= 0x10;
    if (buttons & JP_BUTTON_A2) report[4] |= 0x20;
    if (buttons & JP_BUTTON_DD) report[5] |= 0x01;
    if (buttons & JP_BUTTON_DU) report[5] |= 0x02;
    if (buttons & JP_BUTTON_DR) report[5] |= 0x04;
    if (buttons & JP_BUTTON_DL) report[5] |= 0x08;
    if (buttons & JP_BUTTON_L1) report[5] |= 0x40;
    if (usbd_l2_digital(out, buttons)) report[5] |= 0x80;
    pack_stick(report + 6, axis12(out->left_x), 4095u - axis12(out->left_y));
    pack_stick(report + 9, axis12(out->right_x), 4095u - axis12(out->right_y));
    memset(report + 13, 0, 36);
    if (imu_enabled && out->has_motion) {
        // Joypad's canonical SDL frame matches the PlayStation frame used by
        // Waker's default mapper. Accel is 4096 counts/g; gyro uses raw Sony
        // counts at ±2000 dps, preserving DS4/DS5 motion sensitivity.
        int16_t accel[3], gyro[3];
        int32_t ar = event->accel_range ? event->accel_range : 4000;
        int32_t gr = event->gyro_range ? event->gyro_range : 2000;
        for (unsigned i = 0; i < 3; ++i) {
            accel[i] = clamp16((int64_t)out->accel[i] * ar * 4096 / (32767 * 1000));
            gyro[i] = clamp16((int32_t)out->gyro[i] * gr / 2000);
        }
        for (unsigned base = 13; base <= 37; base += 12) {
            for (unsigned i = 0; i < 3; ++i) {
                put16(report + base + i * 2, accel[i]);
                put16(report + base + 6 + i * 2, gyro[i]);
            }
        }
    }
    last_report_ms = platform_time_ms();
    return tud_hid_report(SW_FULL_REPORT, report + 1, 63);
}

static void switch_mode_handle_output(uint8_t report_id, const uint8_t *data, uint16_t len)
{
    uint8_t packet[SW_REPORT_SIZE] = {0};
    if (!data) return;
    if (report_id) {
        packet[0] = report_id;
        if (len > 63) len = 63;
        memcpy(packet + 1, data, len);
        ++len;
    } else {
        if (len > 64) len = 64;
        memcpy(packet, data, len);
    }
    if (len < 2) return;
    switch (packet[0]) {
    case 0x80:
        if (packet[1] != 0x04) { respond_usb(packet[1]); queue_reply(); }
        break;
    case 0x01:
        sample_rumble(packet, len);
        if (len >= 11) {
            respond_subcommand(packet[10], packet + 11, len - 11);
            queue_reply();
        }
        break;
    case 0x10: sample_rumble(packet, len); break;
    default: break;
    }
}

static bool switch_mode_get_feedback(output_feedback_t *out)
{
    if (!out || !feedback_pending) return false;
    *out = feedback;
    out->dirty = true;
    feedback_pending = false;
    return true;
}

static uint16_t switch_mode_get_report(uint8_t report_id, hid_report_type_t type,
                                        uint8_t *buffer, uint16_t len)
{
    if (type != HID_REPORT_TYPE_INPUT || !buffer) return 0;
    if (report_id == SW_FULL_REPORT || report_id == 0) {
        const uint8_t offset = report_id == 0 ? 0 : 1;
        if (len > SW_REPORT_SIZE - offset) len = SW_REPORT_SIZE - offset;
        memcpy(buffer, report + offset, len);
        return len;
    }
    // Control GET_REPORT must not overwrite a queued interrupt reply.
    uint8_t saved_reply[SW_REPORT_SIZE];
    memcpy(saved_reply, reply, sizeof(saved_reply));
    if (report_id == SW_SUB_REPLY) respond_subcommand(0x02, NULL, 0);
    else if (report_id == SW_USB_REPLY) respond_usb(0x02);
    else return 0;
    if (len > 63) len = 63;
    memcpy(buffer, reply + 1, len);
    memcpy(reply, saved_reply, sizeof(reply));
    return len;
}

static void switch_mode_task(void)
{
    if (!tud_hid_ready()) return;
    if (queued_count) {
        if (tud_hid_report(queued_replies[0][0], queued_replies[0] + 1, 63)) {
            --queued_count;
            for (unsigned i = 0; i < queued_count; ++i)
                memcpy(queued_replies[i], queued_replies[i + 1], SW_REPORT_SIZE);
        }
        return;
    }
    if (platform_time_ms() - last_report_ms < 8) return;
    make_full_report();
    if (tud_hid_report(SW_FULL_REPORT, report + 1, 63)) last_report_ms = platform_time_ms();
}

static const uint8_t *switch_mode_get_device_descriptor(void)
{ return (const uint8_t *)&switch_device_descriptor; }
static const uint8_t *switch_mode_get_config_descriptor(void)
{ return switch_config_descriptor; }
static const uint8_t *switch_mode_get_report_descriptor(void)
{ return switch_report_descriptor; }

const usbd_mode_t switch_mode = {
    .name = "Switch Pro",
    .mode = USB_OUTPUT_MODE_SWITCH,
    .get_device_descriptor = switch_mode_get_device_descriptor,
    .get_config_descriptor = switch_mode_get_config_descriptor,
    .get_report_descriptor = switch_mode_get_report_descriptor,
    .init = switch_mode_init,
    .send_report = switch_mode_send_report,
    .is_ready = switch_mode_is_ready,
    .handle_output = switch_mode_handle_output,
    .get_feedback = switch_mode_get_feedback,
    .get_report = switch_mode_get_report,
    .task = switch_mode_task,
};
