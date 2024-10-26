#ifndef HORIFSD_H_
#define HORIFSD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <tusb.h>
#include <device/usbd_pvt.h>


#define HORIFS_INTERFACE_CLASS 0xFF
#define HORIFS_INTERFACE_SUBCLASS 0x01
#define HORIFS_INTERFACE_PROTOCOL 0x02
#define HORIFS_MAX_PACKET_SIZE 8

#define TUD_HORIFS_DESC_LEN  (9+9+7)

/* input: interrupt data  */
typedef struct __attribute__((packed)) {
  uint8_t stick_x;         /* stick          |  stick (left - right, 0x00 - 0xff) */
  uint8_t stick_y;         /* stick          |  stick (top - bottom, 0x00 - 0xff)  */
  uint8_t rudder;          /* rudder         |  rudder (left - right, 0x00 - 0xff) */
  uint8_t throttle;        /* throttle (top - bottom, 0xff - 0x00)       |  throttle (top - bottom, 0x00 - 0xff) */
  uint8_t hat_x;           /* hat            |  hat (left - right, 0x00 - 0xff) */
  uint8_t hat_y;           /* hat            |  hat (top - bottom, 0x00 - 0xff) */
  uint8_t button_a;        /* triangle       |  button A (press - press, 0x00 - 0xff) */
  uint8_t button_b;        /* square         |  button B (press - press, 0x00 - 0xff) */
} horifs_controller_raw_input_ir;

/* input: vendor request 0x00 data */
typedef struct __attribute__((packed)) {
  uint8_t fire_c    : 1;  /* button select  |  button fire-c */
  uint8_t button_d  : 1;  /* 0x1            |  button D */
  uint8_t hat_btn   : 1;  /* hat press      |  hat press */
  uint8_t button_st : 1;  /* button start   |  button ST */
  uint8_t hat1_U    : 1;  /* d-pad top      |  d-pad 1 top */
  uint8_t hat1_R    : 1;  /* d-pad right    |  d-pad 1 right */
  uint8_t hat1_D    : 1;  /* d-pad bottom   |  d-pad 1 bottom */
  uint8_t hat1_L    : 1;  /* d-pad left     |  d-pad 1 left */

  uint8_t reserved1 : 4;  /* 0xf            |  0xf */
  uint8_t reserved2 : 1;  /* 0x1            |  0x1 */
  uint8_t launch    : 1;  /* button lauch   |  button lauch */
  uint8_t trigger   : 1;  /* trigger        |  trigger */
  uint8_t reserved3 : 1;  /* 0x1            |  0x1 */
} horifs_controller_raw_input_vr_00;

/* input: vendor request 0x01 data */
typedef struct __attribute__((packed)) {
  uint8_t reserved4   : 4; /* 0xf            |        0xf */
  uint8_t hat3_right  : 1; /* 0x1            |        d-pad 3 right */
  uint8_t hat3_middle : 1; /* 0x1            |        d-pad 3 middle */
  uint8_t hat3_left   : 1; /* 0x1            |        d-pad 3 left */
  uint8_t reserved5   : 1; /* 0x1            |        0x0 */

  uint8_t mode_select : 2; /* 0x3            |        mode select (M1 - M2 - M3, 2 - 1 - 3) */
  uint8_t reserved6   : 1; /* 0x1            |        0x1 */
  uint8_t button_sw1  : 1; /* 0x1            |        button sw-1 */
  uint8_t hat2_U      : 1; /* 0x1            |        d-pad 2 top */
  uint8_t hat2_R      : 1; /* 0x1            |        d-pad 2 right */
  uint8_t hat2_D      : 1; /* 0x1            |        d-pad 2 bottom */
  uint8_t hat2_L      : 1; /* 0x1            |        d-pad 2 left */
} horifs_controller_raw_input_vr_01;

/* raw input */
typedef struct __attribute__((packed)) {
    horifs_controller_raw_input_ir ir;
    horifs_controller_raw_input_vr_00 vr_00;
    horifs_controller_raw_input_vr_01 vr_01;
} horifs_controller_raw_input;

#define TUD_HORIFS_DESCRIPTOR(_itfnum, _epin) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, HORIFS_INTERFACE_CLASS, HORIFS_INTERFACE_SUBCLASS, HORIFS_INTERFACE_PROTOCOL, 0x00,\
  /* Unknown */\
  0x09, 0x21, 0x00, 0x01, 0x00, 0x01, 0x22, 0x40, 0x00,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(8), 10


typedef struct
{
    uint8_t itf_num;
    uint8_t ep_in;
    uint8_t rhport;
    uint8_t rumble;
    CFG_TUSB_MEM_ALIGN uint8_t out_buff[1];
    CFG_TUSB_MEM_ALIGN uint8_t in[sizeof(horifs_controller_raw_input_ir)];
    CFG_TUSB_MEM_ALIGN uint8_t r0[sizeof(horifs_controller_raw_input_vr_00)];
    CFG_TUSB_MEM_ALIGN uint8_t r1[sizeof(horifs_controller_raw_input_vr_01)];
} horifs_interface_t;

uint8_t horifs_get_rumble(void);
bool horifs_get_report(uint8_t index, void *report, uint16_t len);
bool horifs_send_report_ready(uint8_t index);
bool horifs_send_report(uint8_t index, horifs_controller_raw_input *report);
const usbd_class_driver_t *horifs_get_driver();

#ifdef __cplusplus
}
#endif

#endif //HORIFSD_H_
