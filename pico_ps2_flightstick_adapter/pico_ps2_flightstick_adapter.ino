/*******************************************************************************
 * Hori FlightStick (FS1/FS2) (PS2) Adapter
 * By Matheus Fraguas (sonik-br)
 * https://github.com/sonik-br/ps2_horifs_adapter
*******************************************************************************/

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "xinput_host.h"
#include "input_usb_host.h"
#include "horifs_device.h"
#include "stick_ids.h"
#include "stick_reports.h"
#include "analog_helper.h"

/*******************************************************************************
 * Config begin
*******************************************************************************/

// USB D+ pin
// D- must be DP +1

// USB host port must use pio-usb:
// Tools -> USB Stack -> Adafruit TinyUSB

//USB D+
#ifndef USE_TINYUSB_HOST
  #if defined(ARDUINO_WAVESHARE_RP2040_PIZERO)
    // The on-board PIO pins on PiZero are known to have problems with some devices. I recommend to use another pins
    #define PIN_USB_HOST_DP 6 // default pin on PiZero
  #elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
    //#define PIN_USB_HOST_DP       16 // Arduino Core already sets this
    #define PIN_USB_HOST_POWER    PIN_5V_EN //18
  #elif defined(ARDUINO_CYTRON_MOTION_2350_PRO)
    //#define PIN_USB_HOST_DP       24 // Arduino Core already sets this
  #else
    #define PIN_USB_HOST_DP       0  // default
  #endif
#endif

// Hori FlightStick model
#define HORIFS_OUTPUT_DEVICE_TYPE 1 // set to 1 or 2

// Force 3d Pro spring effect. Choose one
//#define F3DP_CENTER_SPRING 0x0c, 0x0c, 0x80 //   DEFAULT
//#define F3DP_CENTER_SPRING 0x00, 0x00, 0x00 //   0%
//#define F3DP_CENTER_SPRING 0x05, 0x05, 0x32 //  50%
//#define F3DP_CENTER_SPRING 0x0a, 0x0a, 0x64 // 100%
//#define F3DP_CENTER_SPRING 0x0c, 0x0c, 0xa2 // 120%
//#define F3DP_CENTER_SPRING 0x0d, 0x0d, 0xa2 // 130%
#define F3DP_CENTER_SPRING 0x0f, 0x0f, 0xff // 150%

// Force 3d Pro enable rumble (experimental)
//#define F3DP_ENABLE_RUMBLE

/*******************************************************************************
 * Config end
*******************************************************************************/


// Validation
#ifndef USE_TINYUSB
  #error USB Stack must be configured to use Adafruit TinyUSB
#endif
#ifdef USE_TINYUSB_HOST
  #error USB Stack must NOT be configured as Host (native)
#endif
#ifndef PIN_USB_HOST_DP
  #error Must define PIN_USB_HOST_DP to use PIO-USB
#endif
#if F_CPU != 120000000 && F_CPU != 240000000
  #error PIO USB require CPU Speed must be 120 or 240 MHz
#endif

// report to hold input from any stick
generic_report_t generic_report;

// logitech ffb
enum lg_init_stage_status {
  SENDING_CMDS,
  READY
};

uint8_t mounted_dev = 0;
uint8_t mounted_instance = 0;
uint16_t mounted_vid = 0;
uint16_t mounted_pid = 0;
bool mounted_is_xinput = false;
uint8_t mode_step = 0;
lg_init_stage_status lg_init_stage = SENDING_CMDS;
bool lgl_supports_cmd = false;

horifs_controller_raw_input horifs_data;

class Adafruit_USBD_HoriFS : public Adafruit_USBD_Interface {
  public:
    Adafruit_USBD_HoriFS();
    bool begin(void);
    virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf, uint16_t bufsize);
};

Adafruit_USBD_HoriFS::Adafruit_USBD_HoriFS() { }

bool Adafruit_USBD_HoriFS::begin(void) {
  if (!TinyUSBDevice.addInterface(*this)) {
    return false;
  }

// add to src/arduino/Adafruit_USBD_Device.h
//void setDeviceClass(uint8_t bcd);
//void setDeviceSubClass(uint8_t bcd);
//void setDeviceProtocol(uint8_t bcd);
//void ClearStringIndexes(void);

// add to src/arduino/Adafruit_USBD_Device.cpp
//void Adafruit_USBD_Device::setDeviceClass(uint8_t bcd) {
//  _desc_device.bDeviceClass = bcd;
//}
//void Adafruit_USBD_Device::setDeviceSubClass(uint8_t bcd) {
//  _desc_device.bDeviceSubClass = bcd;
//}
//void Adafruit_USBD_Device::setDeviceProtocol(uint8_t bcd) {
//  _desc_device.bDeviceProtocol = bcd;
//}
//void Adafruit_USBD_Device::ClearStringIndexes(void) {
//  _desc_device.iManufacturer = 0;
//  _desc_device.iProduct = 0;
//  _desc_device.iSerialNumber = 0;
//}

//todo: verify if need #define CFG_TUD_ENDPOINT0_SIZE 8

  TinyUSBDevice.ClearStringIndexes();

  TinyUSBDevice.setID(0x06D3, 0x0F10);
  TinyUSBDevice.setDeviceClass(TUSB_CLASS_VENDOR_SPECIFIC);
  TinyUSBDevice.setDeviceSubClass(0x01);
  TinyUSBDevice.setDeviceProtocol(0xFF);

  #if HORIFS_OUTPUT_DEVICE_TYPE == 1
    // FlightStick 1
    TinyUSBDevice.setVersion(0x0100); 
    TinyUSBDevice.setDeviceVersion(0x0001);
  #else
    // FlightStick 2
    TinyUSBDevice.setVersion(0x0110);
    TinyUSBDevice.setDeviceVersion(0x0002);
  #endif

  return true;
}


uint16_t Adafruit_USBD_HoriFS::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t *buf, uint16_t bufsize) {
  uint8_t itfnum = 0;
  uint8_t ep_in = 0;

  // null buffer is used to get the length of descriptor only
  if (buf) {
    itfnum = TinyUSBDevice.allocInterface(1);
    ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
  }

  const uint8_t desc[] = { TUD_HORIFS_DESCRIPTOR(itfnum, ep_in) };
  const uint16_t len = sizeof(desc); // TUD_HORIFS_DESC_LEN

  if (bufsize < len)
    return 0;

  memcpy(buf, desc, len);
  return len;
}


Adafruit_USBH_Host USBHost;

Adafruit_USBD_HoriFS *_horifs;

void set_led(bool value) {
  #ifdef LED_BUILTIN
    gpio_put(LED_BUILTIN, value);
  #endif
}

void reset_horifs_data() {
  memset(&horifs_data, 0xFF, sizeof(horifs_data));
  horifs_data.ir.stick_x  = 0x80;
  horifs_data.ir.stick_y  = 0x80;
  horifs_data.ir.rudder   = 0x80;
  horifs_data.ir.throttle = 0x80;
  horifs_data.ir.hat_x    = 0x7F;
  horifs_data.ir.hat_y    = 0x7F;
  horifs_data.ir.button_a = 0xD2;
  horifs_data.ir.button_b = 0xD2;
  #if HORIFS_OUTPUT_DEVICE_TYPE == 2 // FlightStick 2
    horifs_data.vr_01.reserved5 = 0;
  #endif
}

void setup() {
  rp2040.enableDoubleResetBootloader();
  Serial.end();
  
  //Configure led pin
  #ifdef LED_BUILTIN
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, 1);
    gpio_put(LED_BUILTIN, LOW);
  #endif

  setup_usb_host();

  reset_horifs_data();

  _horifs = new Adafruit_USBD_HoriFS();
  _horifs->begin();

}

void setup_usb_host() {
  
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  USBHost.begin(1);
}

void loop_usb_host() {
  USBHost.task();

  static uint8_t old_rumble = 0;
  uint8_t current_rumble = horifs_get_rumble();

  // ace combat 5 sends: 0x00 to 0x78. lowest seems to be 0x28

  
  bool update_needed = false;
  if (current_rumble != old_rumble) {
    old_rumble = current_rumble;
    update_needed = true;
  }
  if (update_needed) {
    uint8_t l_rumble = current_rumble;
    uint8_t r_rumble = current_rumble;

    //XBONE have rumble values of 0-100;
    if (mounted_is_xinput) {
      l_rumble = (uint32_t)l_rumble * 100 / 0xFF;
      r_rumble = (uint32_t)r_rumble * 100 / 0xFF;
      
      if (mounted_dev != 0)
        tuh_xinput_set_rumble(mounted_dev, mounted_instance, l_rumble, r_rumble, true);
    }
    else {
      //uint8_t saitek_rumble = map(current_rumble, 0, 255, 0, 127);
      //saitek_send_usb(mounted_dev, 0xB2, saitek_rumble); // use leds. for debug
    }
  }


  // input device connected
  if (mounted_dev && !mounted_is_xinput) {
    static uint32_t last_millis = 0;

    if (lg_init_stage == SENDING_CMDS) {
      const uint8_t cmd_mode[] = {
        0xfe, 0x0d, F3DP_CENTER_SPRING, 0x00, 0x00, /* Configure Spring  */
        0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 /* Enable Spring  */
      };
      const uint8_t mode_cmd_interval = 20; // ms
      const uint8_t mode_cmd_length = 7;
      const uint8_t cmd_count = sizeof(cmd_mode) / mode_cmd_length;

      if (last_millis == 0) { // force an initial delay
        last_millis = millis();
      } else if (millis() - last_millis > 20) { // delay commands
        if (!lgl_supports_cmd) // skip commands
          mode_step = 255;
        if (mode_step < cmd_count) {
          if (tuh_hid_send_report(mounted_dev, mounted_instance, 0, &cmd_mode[7*(mode_step)], 7)) {
            ++mode_step;
          }
        }
        // after all initialization commands are sent, disconnect/reconnect the device to force host re-enumeration
        if (mode_step >= cmd_count) {
          last_millis = 0;
          lg_init_stage = READY; // set next stage
          
          // starts receiving inputs!
          tuh_hid_receive_report(mounted_dev, mounted_instance);
          return;
        }
        last_millis = millis();
      }
    } else { //ready to use
      #ifdef F3DP_ENABLE_RUMBLE
        if (mounted_pid == PID_F3DP) { // if is F3DP
          // apply rumble effect
          const uint8_t rumble_min = 100; //60;
          const uint8_t rumble_max = 120; //90;
          const uint8_t force_update_interval_disable = 8;
          const uint8_t force_update_interval_slow = 48;
          const uint8_t force_update_interval_medium = 32;
          const uint8_t force_update_interval_fast = 16;
    
          static uint8_t rumble_force = rumble_min; // 1 to 127. 1 is maximum force
          static uint32_t last_rumble = 0;
          static bool last_rumble_direction = true;
          static uint8_t force_update_interval = force_update_interval_disable;
    
          static bool rumble_slow = false;
          static bool rumble_fast = false;
    
    
          rumble_slow = old_rumble;
          rumble_fast = false;
          uint8_t temp_force = current_rumble;//max(xpad_rumble.lValue >> 8, xpad_rumble.rValue >> 8); // todo implement
          //rumble_force = map(temp_force, 0, 255, rumble_max, rumble_min); // 1 to 127. 1 is maximum force
          rumble_force = map(temp_force, 20, 120, rumble_max, rumble_min); // 1 to 127. 1 is maximum force
    
//          if (rumble_slow && rumble_fast)
//            force_update_interval = force_update_interval_medium;
//          else if (rumble_slow)
//            force_update_interval = force_update_interval_slow;
//          else if (rumble_fast)
//            force_update_interval = force_update_interval_fast;

          if (old_rumble > 0 && old_rumble <= 40)
            force_update_interval = force_update_interval_slow;
          else if (old_rumble > 40 && old_rumble <= 80)
            force_update_interval = force_update_interval_medium;
          else
            force_update_interval = force_update_interval_fast;

    
          static uint8_t cmd_buffer[7] { 0x00 };
          static uint8_t last_cmd_buffer[7] { 0x00 };
    
          if (rumble_slow || rumble_fast) {
            if (millis() - last_rumble > force_update_interval) {
              cmd_buffer[0] = 0x11; // download and play on slot 1
              cmd_buffer[1] = 0x08; // variable
              cmd_buffer[2] = (last_rumble_direction = !last_rumble_direction) ? -rumble_force: +rumble_force;
              cmd_buffer[3] = 0x80;
    //          cmd_buffer[4] = 0x00;
    //          cmd_buffer[5] = 0x00;
    //          cmd_buffer[6] = 0x00;
              last_rumble =  millis();
            }
          } else {
            if (millis() - last_rumble > force_update_interval_disable) {
              cmd_buffer[0] = 0x13; // stop
              cmd_buffer[1] = 0x00;
              cmd_buffer[2] = 0x00;
              cmd_buffer[3] = 0x00;
    //          cmd_buffer[4] = 0x00;
    //          cmd_buffer[5] = 0x00;
    //          cmd_buffer[6] = 0x00;
              last_rumble =  millis();
            }
          }
    
          // send command to device
          if (memcmp(last_cmd_buffer, cmd_buffer, sizeof(cmd_buffer))) {
            tuh_hid_send_report(mounted_dev, mounted_instance, 0, cmd_buffer, sizeof(cmd_buffer));
            memcpy(last_cmd_buffer, cmd_buffer, sizeof(cmd_buffer));
          }
        } // end if is F3DP
      #endif
    } // end if ready to use
  } // end if is mounted
} // end loop1

void loop() {
//  USBHost.task();
  loop_usb_host();
  
  uint8_t index = 0;
  static horifs_controller_raw_input last_horifs_data {0x00};

  static uint32_t last_report_sent_time = 0;
  uint32_t millis_now = millis();
  
  //if (millis_now - last_report_sent_time > 32 && memcmp(&last_horifs_data, &horifs_data, sizeof(horifs_data)) && horifs_send_report_ready(index) && horifs_send_report(index, &horifs_data)) {
  if (millis_now - last_report_sent_time > 32 && horifs_send_report_ready(index) && horifs_send_report(index, &horifs_data)) {
    //memcpy(&last_horifs_data, &horifs_data, sizeof(horifs_data));
    last_report_sent_time = millis_now;
  }
}// end loop


void map_input(uint8_t const* report) {
  uint16_t vid = mounted_vid;
  uint16_t pid = mounted_pid;

  if (vid == VID_SAITEK && pid == PID_SAITEKX52) { // Saitek X52
    // map the received report to the generic report
    saitekx52_report_t* input_report = (saitekx52_report_t*)report;

    generic_report.stick_x    = input_report->x >> 3;         /* stick          |  stick (left - right, 0x00 - 0xff) */
    generic_report.stick_y    = input_report->y >> 3;         /* stick          |  stick (top - bottom, 0x00 - 0xff)  */
    generic_report.throttle   = ~input_report->throttle;        /* throttle (top - bottom, 0xff - 0x00)       |  throttle (top - bottom, 0x00 - 0xff) */
    generic_report.rudder     = input_report->twist >> 2;          /* rudder         |  rudder (left - right, 0x00 - 0xff) */
    generic_report.hat_x      = map_4_8(input_report->mouse_x);           /* hat            |  hat (left - right, 0x00 - 0xff) */
    generic_report.hat_y      = map_4_8(input_report->mouse_y);           /* hat            |  hat (top - bottom, 0x00 - 0xff) */
    generic_report.button_a   = input_report->c ? ~input_report->slider : 0xff;        /* triangle       |  button A (press - press, 0x00 - 0xff) */
    generic_report.button_b   = input_report->b ? ~input_report->slider : 0xff;        /* square         |  button B (press - press, 0x00 - 0xff) */

    
    generic_report.hat        = (input_report->hat - 1) & 0xF;
  //  generic_report.hat1_U    : 1;  /* d-pad top      |  d-pad 1 top */
  //  generic_report.hat1_R    : 1;  /* d-pad right    |  d-pad 1 right */
  //  generic_report.hat1_D    : 1;  /* d-pad bottom   |  d-pad 1 bottom */
  //  generic_report.hat1_L    : 1;  /* d-pad left     |  d-pad 1 left */
  
    generic_report.fire_c    = input_report->mouse_btn;  /* button select  |  button fire-c */
    generic_report.hat_btn   = input_report->a;  /* hat press      |  hat press */
    generic_report.button_st = input_report->e;  /* button start   |  button ST */
    generic_report.launch    = input_report->fire;  /* button lauch   |  button lauch */
    generic_report.trigger   = input_report->trigger;  /* trigger        |  trigger */
  
    
    /* FlightStick 2 only */
    generic_report.button_d    = input_report->d; /* 0x1            |  button D */
    generic_report.button_sw1  = input_report->pinky; /* 0x1            |  button sw-1 */
    generic_report.hat2_U      = input_report->hat2_U; /* 0x1            |  d-pad 2 top */
    generic_report.hat2_R      = input_report->hat2_R; /* 0x1            |  d-pad 2 right */
    generic_report.hat2_D      = input_report->hat2_D; /* 0x1            |  d-pad 2 bottom */
    generic_report.hat2_L      = input_report->hat2_L; /* 0x1            |  d-pad 2 left */
  //  generic_report.mode_select : 2; /* 0x3            |  mode select (M1 - M2 - M3, 2 - 1 - 3) */
  //  generic_report.hat3_right  : 1; /* 0x1            |  d-pad 3 right */
  //  generic_report.hat3_middle : 1; /* 0x1            |  d-pad 3 middle */
  //  generic_report.hat3_left   : 1; /* 0x1            |  d-pad 3 left */

  } else if (vid == VID_LOGITECH && pid == PID_F3D) { // Logitech Force 3D
    // map the received report to the generic report
    f3d_report_t* input_report = (f3d_report_t*)report;
  } else if ((vid == VID_LOGITECH && pid == PID_F3DP) || (vid == VID_LOGITECH && pid == PID_E3DP)) { // Logitech Force 3D Pro, Extreme 3D Pro
    // map the received report to the generic report
    f3dp_report_t* input_report = (f3dp_report_t*)report;

    // apply deadzone and non-inear scale
    const float deadZone = 30.0f;
    const float antiDeadZone = 0.0f;
    const float linear = 80.0f;
    const bool isInverted = false;
    const bool isHalf = false;
    const bool isThumb = true;

    uint8_t x = round(GetThumbValue(input_report->x >> 2, deadZone, antiDeadZone, linear, isInverted, isHalf, isThumb)) + 128;
    uint8_t y = round(GetThumbValue(input_report->y >> 2, deadZone, antiDeadZone, linear, isInverted, isHalf, isThumb)) + 128;
    uint8_t z = round(GetThumbValue(input_report->twist , 8       , antiDeadZone, 40    , isInverted, isHalf, isThumb)) + 128;
    
//    uint8_t x = dz_sloped_scaled_axial(input_report->x >> 2, 35);
//    uint8_t y = dz_sloped_scaled_axial(input_report->y >> 2, 35);
    //uint8_t z = dz_sloped_scaled_axial(input_report->twist, 10);

    generic_report.stick_x    = x;         /* stick          |  stick (left - right, 0x00 - 0xff) */
    generic_report.stick_y    = y;         /* stick          |  stick (top - bottom, 0x00 - 0xff)  */
    generic_report.throttle   = ~input_report->throttle;        /* throttle (top - bottom, 0xff - 0x00)       |  throttle (top - bottom, 0x00 - 0xff) */
    generic_report.rudder     = z;          /* rudder         |  rudder (left - right, 0x00 - 0xff) */
    generic_report.hat_x      = 0x80; //map_4_8(input_report->mouse_x);           /* hat            |  hat (left - right, 0x00 - 0xff) */
    generic_report.hat_y      = 0x80; //map_4_8(input_report->mouse_y);           /* hat            |  hat (top - bottom, 0x00 - 0xff) */
    generic_report.button_a   = input_report->btn_3 ? 0x00 : 0xff;        /* triangle       |  button A (press - press, 0x00 - 0xff) */
    generic_report.button_b   = input_report->btn_4 ? 0x00 : 0xff;        /* square         |  button B (press - press, 0x00 - 0xff) */

    generic_report.hat        = input_report->hat;

    generic_report.trigger   = input_report->btn_1;  /* trigger        |  trigger */
    generic_report.launch    = input_report->btn_2;  /* button lauch   |  button lauch */

  //  generic_report.hat1_U    : 1;  /* d-pad top      |  d-pad 1 top */
  //  generic_report.hat1_R    : 1;  /* d-pad right    |  d-pad 1 right */
  //  generic_report.hat1_D    : 1;  /* d-pad bottom   |  d-pad 1 bottom */
  //  generic_report.hat1_L    : 1;  /* d-pad left     |  d-pad 1 left */
  
    generic_report.fire_c    = input_report->btn_12;  /* button select  |  button fire-c */
    generic_report.hat_btn   = input_report->btn_5;  /* hat press      |  hat press */
    generic_report.button_st = input_report->btn_11;  /* button start   |  button ST */

  
    /* FlightStick 2 only */
//    generic_report.button_d    = input_report->d; /* 0x1            |  button D */
//    generic_report.button_sw1  = input_report->pinky; /* 0x1            |  button sw-1 */
//    generic_report.hat2_U      = input_report->hat2_U; /* 0x1            |  d-pad 2 top */
//    generic_report.hat2_R      = input_report->hat2_R; /* 0x1            |  d-pad 2 right */
//    generic_report.hat2_D      = input_report->hat2_D; /* 0x1            |  d-pad 2 bottom */
//    generic_report.hat2_L      = input_report->hat2_L; /* 0x1            |  d-pad 2 left */
  //  generic_report.mode_select : 2; /* 0x3            |  mode select (M1 - M2 - M3, 2 - 1 - 3) */
  //  generic_report.hat3_right  : 1; /* 0x1            |  d-pad 3 right */
  //  generic_report.hat3_middle : 1; /* 0x1            |  d-pad 3 middle */
  //  generic_report.hat3_left   : 1; /* 0x1            |  d-pad 3 left */

  } else if (vid == VID_MICROSOFT && pid == PID_SWFFB2 && report[0] == 0x01) { // Microsoft SideWinder FFB2
    // map the received report to output report
    msff2_report_t* input_report = (msff2_report_t*)report;

    uint16_t _x = input_report->x + 512; // maps to unsigned, 10 bits, range 0:1023
    uint16_t _y = input_report->y + 512;
    uint8_t x = _x >> 2; // maps to range 0:255
    uint8_t y = _y >> 2;

    uint8_t _twist = input_report->twist + 32; // maps to unsigneg, 6 bits, range 0:63
    uint8_t twist = (_twist << 2) | (_twist & 0x3); // maps to range 0:255

    //uint8_t throttle = (input_report->throttle << 1) + (input_report->throttle & 1); // maps to 8 bits

    generic_report.stick_x    = x;         /* stick          |  stick (left - right, 0x00 - 0xff) */
    generic_report.stick_y    = y;         /* stick          |  stick (top - bottom, 0x00 - 0xff)  */
    generic_report.throttle   = (~input_report->throttle) * 2;        /* throttle (top - bottom, 0xff - 0x00)       |  throttle (top - bottom, 0x00 - 0xff) */
    generic_report.rudder     = twist;          /* rudder         |  rudder (left - right, 0x00 - 0xff) */
    generic_report.hat_x      = 0x80; //map_4_8(input_report->mouse_x);           /* hat            |  hat (left - right, 0x00 - 0xff) */
    generic_report.hat_y      = 0x80; //map_4_8(input_report->mouse_y);           /* hat            |  hat (top - bottom, 0x00 - 0xff) */
    generic_report.button_a   = input_report->btn_3 ? 0x00 : 0xff;        /* triangle       |  button A (press - press, 0x00 - 0xff) */
    generic_report.button_b   = input_report->btn_4 ? 0x00 : 0xff;        /* square         |  button B (press - press, 0x00 - 0xff) */

    generic_report.hat        = input_report->hat;

    // idea: use btn8 as modifier to use hat as analog hat (camera view)
    // if (input_report->btn_8)

    generic_report.trigger   = input_report->btn_1;  /* trigger        |  trigger */
    generic_report.launch    = input_report->btn_2;  /* button lauch   |  button lauch */

  //  generic_report.hat1_U    : 1;  /* d-pad top      |  d-pad 1 top */
  //  generic_report.hat1_R    : 1;  /* d-pad right    |  d-pad 1 right */
  //  generic_report.hat1_D    : 1;  /* d-pad bottom   |  d-pad 1 bottom */
  //  generic_report.hat1_L    : 1;  /* d-pad left     |  d-pad 1 left */
  
    generic_report.fire_c    = input_report->btn_6;  /* button select  |  button fire-c */
    generic_report.hat_btn   = input_report->btn_5;  /* hat press      |  hat press */
    generic_report.button_st = input_report->btn_7;  /* button start   |  button ST */

  
    /* FlightStick 2 only */
//    generic_report.button_d    = input_report->d; /* 0x1            |  button D */
//    generic_report.button_sw1  = input_report->pinky; /* 0x1            |  button sw-1 */
//    generic_report.hat2_U      = input_report->hat2_U; /* 0x1            |  d-pad 2 top */
//    generic_report.hat2_R      = input_report->hat2_R; /* 0x1            |  d-pad 2 right */
//    generic_report.hat2_D      = input_report->hat2_D; /* 0x1            |  d-pad 2 bottom */
//    generic_report.hat2_L      = input_report->hat2_L; /* 0x1            |  d-pad 2 left */
  //  generic_report.mode_select : 2; /* 0x3            |  mode select (M1 - M2 - M3, 2 - 1 - 3) */
  //  generic_report.hat3_right  : 1; /* 0x1            |  d-pad 3 right */
  //  generic_report.hat3_middle : 1; /* 0x1            |  d-pad 3 middle */
  //  generic_report.hat3_left   : 1; /* 0x1            |  d-pad 3 left */

//    usb_input_device.report.LX = x;
//    usb_input_device.report.LY = y;
//    usb_input_device.report.THROTTLE = ~throttle;
//    usb_input_device.report.TWIST = twist;
//
//    hat_angle_to_UDLR(input_report->hat);
//
//    usb_input_device.report.A = input_report->btn_1;
//    usb_input_device.report.B = input_report->btn_2;
//    usb_input_device.report.X = input_report->btn_3;
//    usb_input_device.report.Y = input_report->btn_4;
//    usb_input_device.report.L1 = input_report->btn_5;
//    usb_input_device.report.R1 = input_report->btn_6;
//    usb_input_device.report.L2 = input_report->btn_7;
//    //usb_input_device.report.R2 = input_report->btn_8;
//    usb_input_device.report.START = input_report->btn_8;

  }
}

void map_output() {

  horifs_data.vr_00.hat1_U = 1;
  horifs_data.vr_00.hat1_D = 1;
  horifs_data.vr_00.hat1_L = 1;
  horifs_data.vr_00.hat1_R = 1;
  switch (generic_report.hat) {
    case 0x0:
      horifs_data.vr_00.hat1_U = 0;
      break;
    case 0x1:
      horifs_data.vr_00.hat1_U = 0;
      horifs_data.vr_00.hat1_R = 0;
      break;
    case 0x2:
      horifs_data.vr_00.hat1_R = 0;
      break;
    case 0x3:
      horifs_data.vr_00.hat1_D = 0;
      horifs_data.vr_00.hat1_R = 0;
      break;
    case 0x4:
      horifs_data.vr_00.hat1_D = 0;
      break;
    case 0x5:
      horifs_data.vr_00.hat1_D = 0;
      horifs_data.vr_00.hat1_L = 0;
      break;
    case 0x6:
      horifs_data.vr_00.hat1_L = 0;
      break;
    case 0x7:
      horifs_data.vr_00.hat1_U = 0;
      horifs_data.vr_00.hat1_L = 0;
      break;
    default:
      break;
  }

  horifs_data.ir.stick_x      = generic_report.stick_x;
  horifs_data.ir.stick_y      = generic_report.stick_y;
  horifs_data.ir.rudder       = generic_report.rudder;
  horifs_data.ir.throttle     = generic_report.throttle;
  horifs_data.ir.hat_x        = generic_report.hat_x;
  horifs_data.ir.hat_y        = generic_report.hat_y;
  horifs_data.ir.button_a     = generic_report.button_a;
  horifs_data.ir.button_b     = generic_report.button_b;

  horifs_data.vr_00.fire_c    = !generic_report.fire_c;  /* button select  |  button fire-c */
  horifs_data.vr_00.hat_btn   = !generic_report.hat_btn;  /* hat press      |  hat press */
  horifs_data.vr_00.button_st = !generic_report.button_st;  /* button start   |  button ST */
  horifs_data.vr_00.launch    = !generic_report.launch;  /* button lauch   |  button lauch */
  horifs_data.vr_00.trigger   = !generic_report.trigger;  /* trigger        |  trigger */

  /* FlightStick 2 only */
  horifs_data.vr_00.button_d    = !generic_report.button_d; /* 0x1            |  button D */
  horifs_data.vr_01.button_sw1  = !generic_report.button_sw1; /* 0x1            |  button sw-1 */
  horifs_data.vr_01.hat2_U      = !generic_report.hat2_U; /* 0x1            |  d-pad 2 top */
  horifs_data.vr_01.hat2_R      = !generic_report.hat2_R; /* 0x1            |  d-pad 2 right */
  horifs_data.vr_01.hat2_D      = !generic_report.hat2_D; /* 0x1            |  d-pad 2 bottom */
  horifs_data.vr_01.hat2_L      = !generic_report.hat2_L; /* 0x1            |  d-pad 2 left */
//  horifs_data.vr_01.mode_select : 2; /* 0x3            |  mode select (M1 - M2 - M3, 2 - 1 - 3) */
//  horifs_data.vr_01.hat3_right  : 1; /* 0x1            |  d-pad 3 right */
//  horifs_data.vr_01.hat3_middle : 1; /* 0x1            |  d-pad 3 middle */
//  horifs_data.vr_01.hat3_left   : 1; /* 0x1            |  d-pad 3 left */

}





void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report_desc, uint16_t desc_len) {
  uint16_t vid;
  uint16_t pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  // device is a supported device
  if ( ((vid == VID_LOGITECH) && ((pid & 0xff00) == 0xc200)) ||
       (vid == VID_SAITEK && pid == PID_SAITEKX52) ||
       (vid == VID_MICROSOFT && pid == PID_SWFFB2)
       ) {

    if (vid == VID_SAITEK && pid == PID_SAITEKX52) {
      saitek_send_usb(dev_addr, 0xB2, 0x00); // turn offleds
    }
    
    set_led(HIGH);

    mode_step = 0;
    lgl_supports_cmd = (vid == VID_LOGITECH && pid == PID_F3DP);
    // set next stage
    lg_init_stage = SENDING_CMDS;


    mounted_vid = vid;
    mounted_pid = pid;
    mounted_dev = dev_addr;
    mounted_instance = idx;
    mounted_is_xinput = false;

    memset(&generic_report, 0, sizeof(generic_report));
    //tuh_hid_receive_report(dev_addr, idx);
  }
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
  set_led(LOW);
  mounted_dev = 0;
  mode_step = 0;
  lg_init_stage = SENDING_CMDS;
  mounted_vid = 0;
  mounted_pid = 0;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len) {

  // safe check
  if (len > 0 && dev_addr == mounted_dev && idx == mounted_instance) {
    // map the received report to generic_output
    map_input(report);

    // now map the generic_output to the output_mode
    map_output();
  }

  // receive next report
  tuh_hid_receive_report(dev_addr, idx);
}


void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, uint16_t len) {
  const xinput_gamepad_t *p = &xid_itf->pad;

  if (xid_itf->last_xfer_result == XFER_RESULT_SUCCESS) {

    if (xid_itf->connected && xid_itf->new_pad_data) {

      uint8_t xboxLX = static_cast<uint16_t>(p->sThumbLX + 32768) >> 8;
      uint8_t xboxLY = static_cast<uint16_t>(p->sThumbLY + 32768) >> 8;
      uint8_t xboxRX = static_cast<uint16_t>(p->sThumbRX + 32768) >> 8;
      uint8_t xboxRY = static_cast<uint16_t>(p->sThumbRY + 32768) >> 8;

      // FS analog data
      horifs_data.ir.stick_x      = xboxLX;
      horifs_data.ir.stick_y      = ~xboxLY;
      horifs_data.ir.rudder       = xboxRX;
      horifs_data.ir.throttle     = xboxRY;
      horifs_data.ir.hat_x        = 0x7F;
      horifs_data.ir.hat_y        = 0x7F;
      horifs_data.ir.button_a     = ~p->bLeftTrigger;
      horifs_data.ir.button_b     = ~p->bRightTrigger;

      // FS digital data
      horifs_data.vr_00.hat1_U = (p->wButtons & XINPUT_GAMEPAD_DPAD_UP)    ? 0 : 1;
      horifs_data.vr_00.hat1_D = (p->wButtons & XINPUT_GAMEPAD_DPAD_DOWN)  ? 0 : 1;
      horifs_data.vr_00.hat1_L = (p->wButtons & XINPUT_GAMEPAD_DPAD_LEFT)  ? 0 : 1;
      horifs_data.vr_00.hat1_R = (p->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) ? 0 : 1;
      horifs_data.vr_00.fire_c    = (p->wButtons & XINPUT_GAMEPAD_BACK)        ? 0 : 1;  /* button select  |  button fire-c */
      horifs_data.vr_00.hat_btn   = (p->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) ? 0 : 1;  /* hat press      |  hat press */
      horifs_data.vr_00.button_st = (p->wButtons & XINPUT_GAMEPAD_START)       ? 0 : 1;  /* button start   |  button ST */
      horifs_data.vr_00.launch    = (p->wButtons & XINPUT_GAMEPAD_B)           ? 0 : 1;  /* button lauch   |  button lauch */
      horifs_data.vr_00.trigger   = (p->wButtons & XINPUT_GAMEPAD_A)           ? 0 : 1;  /* trigger        |  trigger */
    
      /* FlightStick 2 only */
      #if HORIFS_OUTPUT_DEVICE_TYPE == 2
        horifs_data.vr_00.button_d    = 1; /* 0x1            |  button D */
        horifs_data.vr_01.button_sw1  = 1; /* 0x1            |  button sw-1 */
        horifs_data.vr_01.hat2_U      = 1; /* 0x1            |  d-pad 2 top */
        horifs_data.vr_01.hat2_R      = 1; /* 0x1            |  d-pad 2 right */
        horifs_data.vr_01.hat2_D      = 1; /* 0x1            |  d-pad 2 bottom */
        horifs_data.vr_01.hat2_L      = 1; /* 0x1            |  d-pad 2 left */
      #endif
    }
  }
  
  tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf) {
  set_led(HIGH);
  uint16_t PID, VID;
  tuh_vid_pid_get(dev_addr, &VID, &PID);
  
  // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
  // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
  if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false) {
    tuh_xinput_receive_report(dev_addr, instance);
    return;
  }
  tuh_xinput_set_led(dev_addr, instance, 0, true);
  tuh_xinput_set_led(dev_addr, instance, 1, true);
  tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
  tuh_xinput_receive_report(dev_addr, instance);
  mounted_vid = PID;
  mounted_pid = VID;
  mounted_dev = dev_addr;
  mounted_instance = instance;
  mounted_is_xinput = true;
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance) {
  set_led(LOW);
  mounted_dev = 0;
  mounted_vid = 0;
  mounted_pid = 0;
}



bool saitek_send_usb(uint8_t dev_addr, uint16_t index, uint16_t value) {

  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_DEVICE,
          .type      = TUSB_REQ_TYPE_VENDOR,
          .direction = TUSB_DIR_OUT
      },
      .bRequest = 0x91,
      .wValue   = tu_htole16(value),
      .wIndex   = index,
      .wLength  = 0
  };
  tuh_xfer_t xfer = {
      .daddr       = dev_addr,
      .ep_addr     = 0, // control endpoint
      .setup       = &request,
  };

  bool ret = tuh_control_xfer(&xfer);
  return ret;
}

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = *driver_count + 1;
  return horifs_get_driver();
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  bool ret = false;
  ret |= horifs_get_driver()->control_xfer_cb(rhport, stage, request);
  return ret;
}
