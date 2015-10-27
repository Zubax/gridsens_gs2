/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdint>
#include <ch.hpp>
#include <hal.h>
#include "usb_cli.hpp"

namespace usb_cli
{

using std::uint8_t;
using std::uint16_t;

namespace
{
/*
 * VID/PID
 */
constexpr unsigned USB_VID = 0x1d50;
constexpr unsigned USB_PID = 0x60c7;

/*
 * Endpoints to be used for USBD1.
 */
constexpr unsigned USBD1_DATA_REQUEST_EP      = 1;
constexpr unsigned USBD1_DATA_AVAILABLE_EP    = 1;
constexpr unsigned USBD1_INTERRUPT_REQUEST_EP = 2;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] =
{
    USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                           0x02,          /* bDeviceClass (CDC).              */
                           0x00,          /* bDeviceSubClass.                 */
                           0x00,          /* bDeviceProtocol.                 */
                           0x40,          /* bMaxPacketSize.                  */
                           USB_VID,       /* idVendor.                        */
                           USB_PID,       /* idProduct.                       */
                           0x0200,        /* bcdDevice.                       */
                           1,             /* iManufacturer.                   */
                           2,             /* iProduct.                        */
                           3,             /* iSerialNumber.                   */
                           1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor =
{
    sizeof(vcom_device_descriptor_data),
    vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
    /* Configuration Descriptor.*/
    USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                           0x02,          /* bNumInterfaces.                  */
                           0x01,          /* bConfigurationValue.             */
                           0,             /* iConfiguration.                  */
                           0b11000000,    /* bmAttributes (self powered).     */
                           90),           /* bMaxPower.                       */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                           0x00,          /* bAlternateSetting.               */
                           0x01,          /* bNumEndpoints.                   */
                           0x02,          /* bInterfaceClass (Communications
                                             Interface Class, CDC section
                                             4.2).                            */
                           0x02,          /* bInterfaceSubClass (Abstract
                                             Control Model, CDC section 4.3). */
                           0x01,          /* bInterfaceProtocol (AT commands,
                                             CDC section 4.4).                */
                           0),            /* iInterface.                      */
    /* Header Functional Descriptor (CDC section 5.2.3).*/
    USB_DESC_BYTE         (5),            /* bLength.                         */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                             Functional Descriptor.           */
    USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                             Functional Descriptor).          */
    USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
    USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
    /* ACM Functional Descriptor.*/
    USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                             Control Management Descriptor).  */
    USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
    /* Union Functional Descriptor.*/
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                             Functional Descriptor).          */
    USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                             Class Interface).                */
    USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                             Interface).                      */
    /* Endpoint 2 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
                           0x03,          /* bmAttributes (Interrupt).        */
                           0x0008,        /* wMaxPacketSize.                  */
                           0xFF),         /* bInterval.                       */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                           0x00,          /* bAlternateSetting.               */
                           0x02,          /* bNumEndpoints.                   */
                           0x0A,          /* bInterfaceClass (Data Class
                                             Interface, CDC section 4.5).     */
                           0x00,          /* bInterfaceSubClass (CDC section
                                             4.6).                            */
                           0x00,          /* bInterfaceProtocol (CDC section
                                             4.7).                            */
                           0x00),         /* iInterface.                      */
    /* Endpoint 3 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                           0x02,          /* bmAttributes (Bulk).             */
                           0x0040,        /* wMaxPacketSize.                  */
                           0x00),         /* bInterval.                       */
    /* Endpoint 1 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                           0x02,          /* bmAttributes (Bulk).             */
                           0x0040,        /* wMaxPacketSize.                  */
                           0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor =
{
    sizeof(vcom_configuration_descriptor_data),
    vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] =
{
    USB_DESC_BYTE(4),                     /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] =
{
    USB_DESC_BYTE(30),                    /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    'Z', 0, 'u', 0, 'b', 0, 'a', 0, 'x', 0,
    ' ', 0,
    'R', 0, 'o', 0, 'b', 0, 'o', 0, 't', 0, 'i', 0, 'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] =
{
    USB_DESC_BYTE(22),                    /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    'Z', 0, 'u', 0, 'b', 0, 'a', 0, 'x', 0,
    ' ', 0,
    'G', 0, 'N', 0, 'S', 0, 'S', 0
};

/*
 * Serial Number string.
 * Will be configured at run time during initialization.
 */
static uint8_t vcom_string3[] =
{
    USB_DESC_BYTE(66),                    /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, // 8
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, // 16
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, // 24
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0  // 32
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] =
{
    {sizeof(vcom_string0), vcom_string0},
    {sizeof(vcom_string1), vcom_string1},
    {sizeof(vcom_string2), vcom_string2},
    {sizeof(vcom_string3), vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang)
{
    (void) usbp;
    (void) lang;
    switch (dtype)
    {
    case USB_DESCRIPTOR_DEVICE:
    {
        return &vcom_device_descriptor;
    }
    case USB_DESCRIPTOR_CONFIGURATION:
    {
        return &vcom_configuration_descriptor;
    }
    case USB_DESCRIPTOR_STRING:
    {
        if (dindex < 4)
        {
            return &vcom_strings[dindex];
        }
        break;
    }
    default:
    {
        return NULL;
    }
    }
    return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config =
{
    USB_EP_MODE_TYPE_BULK,
    NULL,
    sduDataTransmitted,
    sduDataReceived,
    0x0040,
    0x0040,
    &ep1instate,
    &ep1outstate,
    1,
    NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config =
{
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    0x0010,
    0x0000,
    &ep2instate,
    NULL,
    1,
    NULL
};

/*
 * Serial over USB Driver structure.
 */
static SerialUSBDriver SDU1;

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event)
{
    switch (event)
    {
    case USB_EVENT_RESET:
    {
        return;
    }
    case USB_EVENT_ADDRESS:
    {
        return;
    }
    case USB_EVENT_CONFIGURED:
    {
        chSysLockFromIsr();

        /*
         * Enables the endpoints specified into the configuration.
         * Note, this callback is invoked from an ISR so I-Class functions
         * must be used.
         */
        usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
        usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

        /* Resetting the state of the CDC subsystem.*/
        sduConfigureHookI(&SDU1);

        chSysUnlockFromIsr();
        return;
    }
    case USB_EVENT_SUSPEND:
    {
        return;
    }
    case USB_EVENT_WAKEUP:
    {
        return;
    }
    case USB_EVENT_STALLED:
    {
        return;
    }
    }
}

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg =
{
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
};

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg =
{
    &USBD1,
    USBD1_DATA_REQUEST_EP,
    USBD1_DATA_AVAILABLE_EP,
    USBD1_INTERRUPT_REQUEST_EP
};

}

void init(const DeviceSerialNumber& device_serial)
{
    /*
     * Initializing the device serial
     */
    {
        static const auto nibble2hex = [](std::uint8_t x)
        {
            const char n = char((x & 0xF) + '0');
            return (n > '9') ? char(n + 'A' - '9' - 1) : n;
        };

        unsigned write_pos = 2;
        for (auto x : device_serial)
        {
            vcom_string3[write_pos++] = nibble2hex(x >> 4);
            write_pos++;
            vcom_string3[write_pos++] = nibble2hex(x);
            write_pos++;
        }
    }

    /*
     * Initializing the serial over USB driver
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activate the USB driver and then the USB bus pull-up on D+.
     * A delay is inserted in order to not have to disconnect the cable after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepSeconds(1);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

SerialUSBDriver* getSerialUSBDriver()
{
    return &SDU1;
}

bool isConnected()
{
    return SDU1.config->usbp->state == USB_ACTIVE;
}

}
