#pragma once

// generic report
typedef struct TU_ATTR_PACKED {
  uint8_t stick_x;         /* stick          |  stick (left - right, 0x00 - 0xff) */
  uint8_t stick_y;         /* stick          |  stick (top - bottom, 0x00 - 0xff)  */
  uint8_t throttle;        /* throttle (top - bottom, 0xff - 0x00)       |  throttle (top - bottom, 0x00 - 0xff) */
  uint8_t rudder;          /* rudder         |  rudder (left - right, 0x00 - 0xff) */
  uint8_t hat_x;           /* hat            |  hat (left - right, 0x00 - 0xff) */
  uint8_t hat_y;           /* hat            |  hat (top - bottom, 0x00 - 0xff) */
  uint8_t button_a;        /* triangle       |  button A (press - press, 0x00 - 0xff) */
  uint8_t button_b;        /* square         |  button B (press - press, 0x00 - 0xff) */

  uint8_t hat1_U    : 1;  /* d-pad top      |  d-pad 1 top */
  uint8_t hat1_D    : 1;  /* d-pad bottom   |  d-pad 1 bottom */
  uint8_t hat1_L    : 1;  /* d-pad left     |  d-pad 1 left */
  uint8_t hat1_R    : 1;  /* d-pad right    |  d-pad 1 right */
  uint8_t trigger   : 1;  /* trigger        |  trigger */
  uint8_t launch    : 1;  /* button lauch   |  button lauch */
  uint8_t fire_c    : 1;  /* button select  |  button fire-c */
  uint8_t button_st : 1;  /* button start   |  button ST */
  uint8_t hat_btn   : 1;  /* hat press      |  hat press */

  /* FlightStick 2 only */
  uint8_t button_d    : 1; /* 0x1            |  button D */
  uint8_t button_sw1  : 1; /* 0x1            |  button sw-1 */
  uint8_t hat2_U      : 1; /* 0x1            |  d-pad 2 top */
  uint8_t hat2_D      : 1; /* 0x1            |  d-pad 2 bottom */
  uint8_t hat2_L      : 1; /* 0x1            |  d-pad 2 left */
  uint8_t hat2_R      : 1; /* 0x1            |  d-pad 2 right */
  uint8_t hat3_left   : 1; /* 0x1            |  d-pad 3 left */
  uint8_t hat3_middle : 1; /* 0x1            |  d-pad 3 middle */
  uint8_t hat3_right  : 1; /* 0x1            |  d-pad 3 right */
  uint8_t mode_select : 2; /* 0x3            |  mode select (M1 - M2 - M3, 2 - 1 - 3) */
  uint8_t hat : 4;


//  uint8_t pedals_precision_16bits : 1; // pedals uses 16bits?
//  //wheel_precision wheel_precision : 2; // wheel precision enum
//  uint8_t wheel_precision : 2; // wheel precision

} generic_report_t;










// Logitech Force 3D
typedef struct TU_ATTR_PACKED {
  uint8_t x; // left ~0x4b, middle ~0x80, right ~0xb1
  uint8_t y; // top ~0x4e, middle ~0x80, bottom ~0xb9
  uint8_t : 4;  // always 0 (vendor)
  uint8_t hat : 4;
  uint8_t twist; // left ~0x75, middle ~0xcd, right 0xff
  uint8_t btn_1 : 1; // trigger
  uint8_t btn_2 : 1;
  uint8_t btn_3 : 1;
  uint8_t btn_4 : 1;
  uint8_t btn_5 : 1;
  uint8_t btn_6 : 1;
  uint8_t btn_7 : 1;
  uint8_t : 1; // always 0 (padding)
  uint8_t throttle; // up ~0x1b, middle ~0x66, down ~0xb5
  uint8_t unknown; // allways 0x03 (vendor)
} f3d_report_t;

// Logitech Force 3D Pro
typedef struct TU_ATTR_PACKED {
  uint16_t x : 10;
  uint16_t y : 10;
  uint8_t hat : 4;
  uint8_t twist; // left 0x00, middle 0x80, right 0xff
  uint8_t btn_1 : 1; // trigger
  uint8_t btn_2 : 1;
  uint8_t btn_3 : 1;
  uint8_t btn_4 : 1;
  uint8_t btn_5 : 1;
  uint8_t btn_6 : 1;
  uint8_t btn_7 : 1;
  uint8_t btn_8 : 1;
  uint8_t throttle; // up 0x00, middle 0x80, down 0xff
  uint8_t btn_9 : 1;
  uint8_t btn_10 : 1;
  uint8_t btn_11 : 1;
  uint8_t btn_12 : 1;
  uint8_t : 4; // allways 0
} f3dp_report_t;

// Logitech Extreme 3D Pro. report looks to be exactly the same as the force 3d pro
// struct from https://github.com/touchgadget/switch_onehand
//typedef struct TU_ATTR_PACKED {
//  uint16_t x : 10;      // 0..512..1023
//  uint16_t y : 10;      // 0..512..1023
//  uint8_t hat : 4;
//  uint8_t twist : 8;   // 0..127..255
//  uint8_t btn_1 : 1; // trigger
//  uint8_t btn_2 : 1;
//  uint8_t btn_3 : 1;
//  uint8_t btn_4 : 1;
//  uint8_t btn_5 : 1;
//  uint8_t btn_6 : 1;
//  uint8_t btn_7 : 1;
//  uint8_t btn_8 : 1;
//  uint8_t throttle;       // 0..255
//  uint8_t btn_9 : 1;
//  uint8_t btn_10 : 1;
//  uint8_t btn_11 : 1;
//  uint8_t btn_12 : 1;
//  uint8_t : 4; // no idea
//} e3dp_report_t;



// Saitek X52
typedef struct TU_ATTR_PACKED {
  uint16_t x : 11; // (X)
  uint16_t y : 11; // (Y)
  uint16_t twist : 10; // (RZ) left 00, right ff

  uint8_t throttle; // (Z) up 00, down ff
  uint8_t rotary_i; // (Rx) left 00, right ff
  uint8_t rotary_e; // (Ry) left 00, right ff
  uint8_t slider; // (Slider) left 00, right ff
  
  //uint32_t buttons;
  uint8_t trigger : 1;
  uint8_t fire : 1;
  uint8_t a  : 1;
  uint8_t b  : 1;
  uint8_t c  : 1;
  uint8_t pinky : 1;
  uint8_t d : 1;
  uint8_t e : 1;
  
  uint8_t t1 : 1;
  uint8_t t2 : 1;
  uint8_t t3 : 1;
  uint8_t t4 : 1;
  uint8_t t5 : 1;
  uint8_t t6 : 1;
  uint8_t trigger_full : 1;
  uint8_t hat2_U : 1;
  
  uint8_t hat2_R : 1;
  uint8_t hat2_D : 1;
  uint8_t hat2_L : 1;
  uint8_t hat3_U : 1;
  uint8_t hat3_R : 1;
  uint8_t hat3_D : 1;
  uint8_t hat3_L : 1;
  uint8_t mode1 : 1;
  
  uint8_t mode2 : 1;
  uint8_t mode3 : 1;
  uint8_t function : 1;
  uint8_t startstop : 1;
  uint8_t reset : 1;
  uint8_t i : 1; // clutch button
  uint8_t mouse_btn : 1;
  uint8_t scroll_btn : 1;

  uint8_t scroll_down : 1;
  uint8_t scroll_up : 1;
  uint8_t : 1;
  uint8_t : 1;
  uint8_t hat : 4; // rest 0x0, top 0x1

  uint8_t mouse_x : 4; // left 0, mid 8, right f
  uint8_t mouse_y : 4; // top 0, mid 8, bottol f
} saitekx52_report_t;

// Microsoft Sidewinder FFB 2
typedef struct TU_ATTR_PACKED {
  uint8_t report_id;
  uint16_t x; // (-512 to 511)
  uint16_t y; // (-512 to 511)
  uint8_t twist; // left 0xe0, middle 0x00, right 0xlf (-32 o 31)
  uint8_t throttle; // up 0x00, middle 0x40, down 0x7f
  uint8_t hat : 4;
  uint8_t : 4;
  uint8_t btn_1 : 1; // trigger
  uint8_t btn_2 : 1;
  uint8_t btn_3 : 1;
  uint8_t btn_4 : 1;
  uint8_t btn_5 : 1;
  uint8_t btn_6 : 1;
  uint8_t btn_7 : 1;
  uint8_t btn_8 : 1;
  uint16_t force_x; // unknown. seems to be related to ffb
  uint16_t force_y; // unknown. seems to be related to ffb
} msff2_report_t;
