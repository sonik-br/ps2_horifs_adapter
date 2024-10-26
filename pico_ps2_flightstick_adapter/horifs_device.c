#include "horifs_device.h"

#define MAX_HFS 1

CFG_TUSB_MEM_SECTION static horifs_interface_t _hfs_itf[MAX_HFS];

static inline int8_t get_index_by_itfnum(uint8_t itf_num)
{
  return 0;
//    for (uint8_t i = 0; i < MAX_HFS; i++)
//    {
//        if (itf_num == _hfs_itf[i].itf_num)
//            return i;
//    }
//    return -1;
}

static inline int8_t get_index_by_ep_addr(uint8_t ep_addr)
{
  return 0;
//    for (uint8_t i = 0; i < MAX_HFS; i++)
//    {
//        if (ep_addr == _hfs_itf[i].ep_in)
//            return i;
//
////        if (ep_addr == _hfs_itf[i].ep_out)
////            return i;
//    }
//    return -1;
}

static inline horifs_interface_t *find_available_interface()
{
    for (uint8_t i = 0; i < MAX_HFS; i++)
    {
        if (_hfs_itf[i].ep_in == 0)
            return &_hfs_itf[i];
    }
    return NULL;
}

static void horifs_init(void)
{
    tu_memclr(_hfs_itf, sizeof(_hfs_itf));
    for (uint8_t i = 0; i < MAX_HFS; i++)
    {
      _hfs_itf[i].rumble = 0;
    }
}

static void horifs_reset(uint8_t rhport)
{
    horifs_init();
}

static uint16_t horifs_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    TU_VERIFY(itf_desc->bInterfaceClass == HORIFS_INTERFACE_CLASS, 0xFF);
    TU_VERIFY(itf_desc->bInterfaceSubClass == HORIFS_INTERFACE_SUBCLASS, 0x01);

    horifs_interface_t *p_hfs = find_available_interface();
    TU_ASSERT(p_hfs != NULL, 0);

    uint16_t const drv_len = TUD_HORIFS_DESC_LEN;
    TU_ASSERT(max_len >= drv_len, 0);

    p_hfs->itf_num = itf_desc->bInterfaceNumber;
    p_hfs->rhport = rhport;

    tusb_desc_endpoint_t *ep_desc;
    ep_desc = (tusb_desc_endpoint_t *)tu_desc_next(itf_desc);

    if (tu_desc_type(ep_desc) == TUSB_DESC_ENDPOINT)
    {
        usbd_edpt_open(rhport, ep_desc);
//        (ep_desc->bEndpointAddress & 0x80) ? (p_hfs->ep_in  = ep_desc->bEndpointAddress) :
//                                             (p_hfs->ep_out = ep_desc->bEndpointAddress);
        p_hfs->ep_in  = ep_desc->bEndpointAddress;
    }

    TU_VERIFY(itf_desc->bNumEndpoints >= 1, drv_len);
    ep_desc = (tusb_desc_endpoint_t *)tu_desc_next(ep_desc);

    if (tu_desc_type(ep_desc) == TUSB_DESC_ENDPOINT)
    {
        usbd_edpt_open(rhport, ep_desc);
//        (ep_desc->bEndpointAddress & 0x80) ? (p_hfs->ep_in  = ep_desc->bEndpointAddress) :
//                                             (p_hfs->ep_out = ep_desc->bEndpointAddress);
        p_hfs->ep_in  = ep_desc->bEndpointAddress;
    }
    return drv_len;
}

uint8_t horifs_get_rumble(void)
{
  return _hfs_itf[0].rumble;
}

bool horifs_get_report(uint8_t index, void *report, uint16_t len)
{
    TU_VERIFY(index < MAX_HFS, false);
//    TU_VERIFY(_hfs_itf[index].ep_out != 0, false);
//    TU_VERIFY(len < HORIFS_MAX_PACKET_SIZE, false);

//    memcpy(report, _hfs_itf[index].out, len);

    if (tud_ready() && !usbd_edpt_busy(_hfs_itf[index].rhport, _hfs_itf[index].ep_in))
    {
        usbd_edpt_xfer(_hfs_itf[index].rhport, _hfs_itf[index].ep_in, _hfs_itf[index].in, len);
    }
    return true;
}

bool horifs_send_report_ready(uint8_t index)
{
    TU_VERIFY(index < MAX_HFS, false);
    TU_VERIFY(_hfs_itf[index].ep_in != 0, false);

    return (tud_ready() && !usbd_edpt_busy(_hfs_itf[index].rhport, _hfs_itf[index].ep_in));
}
bool horifs_send_report(uint8_t index, horifs_controller_raw_input *report)
{
//    TU_VERIFY(len < HORIFS_MAX_PACKET_SIZE, false);
    TU_VERIFY(index < MAX_HFS, false);
    TU_VERIFY(_hfs_itf[index].ep_in != 0, false);
    TU_VERIFY(horifs_send_report_ready(index), false);

    if (tud_suspended())
        tud_remote_wakeup();

    //Maintain a local copy of the report
    memcpy(_hfs_itf[index].in, &report->ir, sizeof(horifs_controller_raw_input_ir));
    memcpy(_hfs_itf[index].r0, &report->vr_00, sizeof(horifs_controller_raw_input_vr_00));
    memcpy(_hfs_itf[index].r1, &report->vr_01, sizeof(horifs_controller_raw_input_vr_01));

    //Send interrupt data to the host
    return usbd_edpt_xfer(_hfs_itf[index].rhport, _hfs_itf[index].ep_in, _hfs_itf[index].in, sizeof(horifs_controller_raw_input_ir));
}

static bool horifs_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport;
    uint8_t index = get_index_by_ep_addr(ep_addr);

    TU_VERIFY(result == XFER_RESULT_SUCCESS, true);
    TU_VERIFY(index != -1, true);
    TU_VERIFY(xferred_bytes < HORIFS_MAX_PACKET_SIZE, true);

//    if (ep_addr == _hfs_itf[index].ep_out)
//    {
//        memcpy(_hfs_itf[index].out, _hfs_itf[index].ep_out_buff, MIN(xferred_bytes, sizeof( _hfs_itf[index].ep_out_buff)));
//    }

    return true;
}

bool horifs_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{

    TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE || request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE);

    uint8_t index = get_index_by_itfnum((uint8_t)request->wIndex);
    TU_VERIFY(index != -1, false);

    bool ret = false;

    // Report data 00
    if (request->bmRequestType_bit.direction == TUSB_DIR_IN && request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->wLength == 2 && request->bRequest == 0x00)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
          tud_control_xfer(rhport, request, _hfs_itf[index].r0, MIN(request->wLength, sizeof(_hfs_itf[index].in)));
        }
        return true;
    }

    // Report data 01
    if (request->bmRequestType_bit.direction == TUSB_DIR_IN && request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->wLength == 2 && request->bRequest == 0x01)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
          tud_control_xfer(rhport, request, _hfs_itf[index].r1, MIN(request->wLength, sizeof(_hfs_itf[index].in)));
        }
        return true;
    }

    // Rumble
    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE && request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bmRequestType_bit.direction == TUSB_DIR_OUT && request->wLength == 1 && request->bRequest == 0x0c)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
            //Host is sending a rumble command to control pipe. Queue receipt.
            tud_control_xfer(rhport, request, _hfs_itf[index].out_buff, MIN(request->wLength, sizeof(_hfs_itf[index].out_buff)));
        }
        else if (stage == CONTROL_STAGE_ACK)
        {
            //Receipt complete. Copy data to rumble struct
            TU_LOG1("Got HID report from control pipe for index %02x\n", request->wIndex);
            //memcpy(&_hfs_itf[index].rumble, _hfs_itf[index].out_buff, MIN(request->wLength, sizeof(_hfs_itf[index].rumble)));
            _hfs_itf[index].rumble = _hfs_itf[index].out_buff[0];
        }
        return true;
    }

    // Unknown
    if (request->bmRequestType_bit.direction == TUSB_DIR_OUT && request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD && request->wIndex == 0 && request->wLength == 0 && request->bRequest == 0x0b && request->wValue == 0x0000)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
          uint8_t txbuftemp[2] = {0xff, 0xff};
          return tud_control_xfer(rhport, request, txbuftemp, 0);
        }
        return true;
    }

    if (ret == false)
    {
        TU_LOG1("STALL: wIndex: %02x bmRequestType: %02x, bRequest: %02x, wValue: %04x\n",
                request->wIndex,
                request->bmRequestType,
                request->bRequest,
                request->wValue);
        return false;
    }

    return true;
}

static const usbd_class_driver_t horifs_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name = "HORIFS DRIVER",
#endif
    .init = horifs_init,
    .reset = horifs_reset,
    .open = horifs_open,
    .control_xfer_cb = horifs_control_xfer_cb,
    .xfer_cb = horifs_xfer_cb,
    .sof = NULL
};

const usbd_class_driver_t *horifs_get_driver()
{
    return &horifs_driver;
}
