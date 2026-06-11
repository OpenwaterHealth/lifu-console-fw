#include "usb_events.h"

/* One slot per event — indexed by usb_event_t. */
static usb_event_cb_t cb_table[USB_EVENT_COUNT] = { 0 };

void usb_register_callback(usb_event_t event, usb_event_cb_t cb)
{
    if (event < USB_EVENT_COUNT)
    {
        cb_table[event] = cb;
    }
}

void usb_notify(usb_event_t event)
{
    if (event < USB_EVENT_COUNT && cb_table[event])
    {
        cb_table[event](event);
    }
}
