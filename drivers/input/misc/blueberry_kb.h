#ifndef _ASUSDEC_H
#define _ASUSDEC_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define ASUSDEC_DEBUG               0
#define MY_DBG                      1

#define HID_MATRIX		1	// 0: scancode2, 1: hid

/*
 * Debug Utility
 */
#if ASUSDEC_DEBUG
#define ASUSDEC_INFO(format, arg...)    \
    printk(KERN_INFO "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)
#define ASUSDEC_I2C_DATA(array, i)  \
                    do {        \
                        for (i = 0; i < array[0]+1; i++) \
                            ASUSDEC_INFO("ec_data[%d] = 0x%x\n", i, array[i]);  \
                    } while(0)
#else
#define ASUSDEC_INFO(format, arg...)     
#define ASUSDEC_I2C_DATA(array, i)
#endif

#define ASUSDEC_NOTICE(format, arg...)  \
    printk(KERN_NOTICE "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSDEC_ERR(format, arg...) \
    printk(KERN_ERR "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)

/////////////////////////////////////////////////////////////
struct key_map{
	u8 value;
	int key_code;
};

struct key_map blueberry_key_map[]=
{
    {0x6e,  KEY_ESC},
    {0x70,  KEY_F1},
    {0x71,  KEY_F2},
    {0x72,  KEY_F3},
    {0x73,   KEY_F4},
    {0x74,  KEY_F5},
    {0x75,   KEY_F6},
    {0x76,  KEY_F7},
    {0x77,  KEY_F8},
    {0x78,  KEY_F9},
    {0x79,  KEY_F10},
    {0x7a,  KEY_F11},
    {0x7b,  KEY_F12},

    {0x1,  KEY_GRAVE},//~
    {0x2,  KEY_1},
    {0x3,  KEY_2},
    {0x4,  KEY_3},
    {0x5,  KEY_4},
    {0x6,  KEY_5},
    {0x7,  KEY_6},
    {0x8,  KEY_7},        
    {0x9,  KEY_8},   
    {0xa,  KEY_9},        
    {0xb,  KEY_0},   
    {0xc,  KEY_MINUS},
    {0xd,  KEY_EQUAL},
    {0xf,  KEY_BACKSPACE},

    {0x10,  KEY_TAB},
    {0x11,  KEY_Q},
    {0x12,  KEY_W},
    {0x13,  KEY_E},
    {0x14,  KEY_R},
    {0x15,  KEY_T},
    {0x16,  KEY_Y},
    {0x17,  KEY_U},
    {0x18,  KEY_I},
    {0x19,  KEY_O},
    {0x1a,  KEY_P},
    {0x1b,  KEY_LEFTBRACE},
    {0x1c,  KEY_RIGHTBRACE},
    {0x1d,  KEY_SPORT},

    {0x1e,   KEY_CAPSLOCK},
    {0x1f,   KEY_A},
    {0x20,   KEY_S},
    {0x21,   KEY_D},
    {0x22,   KEY_F},
    {0x23,   KEY_G},
    {0x24,   KEY_H},
    {0x25,   KEY_J},
    {0x26,   KEY_K},
    {0x27,   KEY_L},
    {0x28,   KEY_SEMICOLON},
    {0x29,   KEY_APOSTROPHE},
    {0x2b,   KEY_ENTER},

    {0x2c,   KEY_LEFTSHIFT},
    {0x2e,   KEY_Z},
    {0x2f,   KEY_X},
    {0x30,   KEY_C},
    {0x31,   KEY_V},
    {0x32,   KEY_B},
    {0x33,   KEY_N},
    {0x34,   KEY_M},
    {0x45,   KEY_COMMA},
    {0x36,   KEY_DOT},
    {0x37,   KEY_SLASH},
    {0x39,   KEY_RIGHTSHIFT},

    {0x3a,   KEY_FN},
    {0x3b,   KEY_LEFTCTRL},
    {0x7f,   KEY_LEFTMETA},
    {0x3c,   KEY_LEFTALT},
    {0x3d,   KEY_SPACE},
    {0x3e,   KEY_RIGHTALT},
    {0x40,   KEY_DELETE},
    {0x4f,   KEY_LEFT},
    {0x53,   KEY_UP},
    {0x54,   KEY_DOWN},
    {0x59,   KEY_RIGHT},   

};

#if	HID_MATRIX

#define ASUSDEC_KEYPAD_ESC              0x29
#define ASUSDEC_KEYPAD_KEY_WAVE         0x35// `~
#define ASUSDEC_KEYPAD_KEY_1            0x1e
#define ASUSDEC_KEYPAD_KEY_2            0X1f
#define ASUSDEC_KEYPAD_KEY_3            0x20    
#define ASUSDEC_KEYPAD_KEY_4            0x21
#define ASUSDEC_KEYPAD_KEY_5            0x22
#define ASUSDEC_KEYPAD_KEY_6            0x23
#define ASUSDEC_KEYPAD_KEY_7            0x24
#define ASUSDEC_KEYPAD_KEY_8            0x25
#define ASUSDEC_KEYPAD_KEY_9            0x26
#define ASUSDEC_KEYPAD_KEY_0            0x27
#define ASUSDEC_KEYPAD_KEY_MINUS        0x2d// -_
#define ASUSDEC_KEYPAD_KEY_EQUAL        0x2e// =+
#define ASUSDEC_KEYPAD_KEY_BACKSPACE    0x2a
#define ASUSDEC_KEYPAD_KEY_TAB          0x2b
#define ASUSDEC_KEYPAD_KEY_Q            0x14
#define ASUSDEC_KEYPAD_KEY_W            0x1a
#define ASUSDEC_KEYPAD_KEY_E            0x08
#define ASUSDEC_KEYPAD_KEY_R            0x15
#define ASUSDEC_KEYPAD_KEY_T            0x17
#define ASUSDEC_KEYPAD_KEY_Y            0x1c
#define ASUSDEC_KEYPAD_KEY_U            0x18
#define ASUSDEC_KEYPAD_KEY_I            0x0c
#define ASUSDEC_KEYPAD_KEY_O            0x12
#define ASUSDEC_KEYPAD_KEY_P            0x13
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE    0x2f// [{
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE   0x30// ]}
#define ASUSDEC_KEYPAD_KEY_BACKSLASH    0x31// \|
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK     0x39
#define ASUSDEC_KEYPAD_KEY_A            0x04
#define ASUSDEC_KEYPAD_KEY_S            0x16
#define ASUSDEC_KEYPAD_KEY_D            0x07
#define ASUSDEC_KEYPAD_KEY_F            0x09
#define ASUSDEC_KEYPAD_KEY_G            0x0a
#define ASUSDEC_KEYPAD_KEY_H            0x0b
#define ASUSDEC_KEYPAD_KEY_J            0x0d
#define ASUSDEC_KEYPAD_KEY_K            0x0e
#define ASUSDEC_KEYPAD_KEY_L            0x0f
#define ASUSDEC_KEYPAD_KEY_SEMICOLON    0x33// ;:
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE   0x34// '"
#define ASUSDEC_KEYPAD_KEY_ENTER        0x28
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT    0xe1
#define ASUSDEC_KEYPAD_KEY_Z            0x1d
#define ASUSDEC_KEYPAD_KEY_X            0x1b
#define ASUSDEC_KEYPAD_KEY_C            0x06
#define ASUSDEC_KEYPAD_KEY_V            0x19
#define ASUSDEC_KEYPAD_KEY_B            0x05
#define ASUSDEC_KEYPAD_KEY_N            0x11
#define ASUSDEC_KEYPAD_KEY_M            0x10
#define ASUSDEC_KEYPAD_KEY_COMMA        0x36// ,<
#define ASUSDEC_KEYPAD_KEY_DOT          0x37// .>
#define ASUSDEC_KEYPAD_KEY_SLASH        0x38// ?/
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0xe5

#define ASUSDEC_KEYPAD_KEY_LEFT         0x50
#define ASUSDEC_KEYPAD_KEY_RIGHT        0x4f
#define ASUSDEC_KEYPAD_KEY_UP           0x52
#define ASUSDEC_KEYPAD_KEY_DOWN         0x51
#define ASUSDEC_KEYPAD_RIGHTWIN         0xe7// Right GUI
#define ASUSDEC_KEYPAD_LEFTCTRL         0xe0
#define ASUSDEC_KEYPAD_LEFTWIN          0xe3// Left GUI
#define ASUSDEC_KEYPAD_LEFTALT          0xe2
#define ASUSDEC_KEYPAD_KEY_SPACE        0x2c
#define ASUSDEC_KEYPAD_RIGHTALT         0xe6
#define ASUSDEC_KEYPAD_WINAPP           0x65
#define ASUSDEC_KEYPAD_RIGHTCTRL        0xe4
#define ASUSDEC_KEYPAD_HOME             0x4a
#define ASUSDEC_KEYPAD_PAGEUP           0x4b
#define ASUSDEC_KEYPAD_PAGEDOWN         0x4e
#define ASUSDEC_KEYPAD_END              0x4d

#define ASUSDEC_KEYPAD_F1				0x3a
#define ASUSDEC_KEYPAD_F2				0x3b
#define ASUSDEC_KEYPAD_F3				0x3c
#define ASUSDEC_KEYPAD_F4				0x3d
#define ASUSDEC_KEYPAD_F5				0x3e
#define ASUSDEC_KEYPAD_F6				0x3f
#define ASUSDEC_KEYPAD_F7				0x40
#define ASUSDEC_KEYPAD_F8				0x41
#define ASUSDEC_KEYPAD_F9				0x42
#define ASUSDEC_KEYPAD_F10				0x43
#define ASUSDEC_KEYPAD_F11				0x44
#define ASUSDEC_KEYPAD_F12				0x45
#define ASUSDEC_KEYPAD_INSERT			0x49
#define ASUSDEC_KEYPAD_PRINTSCREEN		0x46
#define ASUSDEC_KEYPAD_DELETE			0x4c

//OEM KEY
#define KEY_LAUNCHER		250
#define KEY_SETTING		251
#define KEY_TOUCHPAD		252
#define KEY_FORCE_ROTATION	253
#define KEY_SCREENSHOT		254

/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU         0x94                
#define ASUSDEC_YEN                     0x89
#define ASUSDEC_MUHENKAN                0x8b        
#define ASUSDEC_HENKAN                  0x8a        
#define ASUSDEC_HIRAGANA_KATAKANA       0x88
#define ASUSDEC_RO                      0x87
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2                0x64
/********************************/

#else

/*************scan 2 make mapping***************/
#define ASUSDEC_KEYPAD_ESC              0x76
#define ASUSDEC_KEYPAD_KEY_WAVE         0x0E
#define ASUSDEC_KEYPAD_KEY_1            0x16
#define ASUSDEC_KEYPAD_KEY_2            0X1E
#define ASUSDEC_KEYPAD_KEY_3            0x26    
#define ASUSDEC_KEYPAD_KEY_4            0x25
#define ASUSDEC_KEYPAD_KEY_5            0x2E
#define ASUSDEC_KEYPAD_KEY_6            0x36
#define ASUSDEC_KEYPAD_KEY_7            0x3D
#define ASUSDEC_KEYPAD_KEY_8            0x3E
#define ASUSDEC_KEYPAD_KEY_9            0x46
#define ASUSDEC_KEYPAD_KEY_0            0x45
#define ASUSDEC_KEYPAD_KEY_MINUS        0x4E
#define ASUSDEC_KEYPAD_KEY_EQUAL        0x55
#define ASUSDEC_KEYPAD_KEY_BACKSPACE    0x66
#define ASUSDEC_KEYPAD_KEY_TAB          0x0D
#define ASUSDEC_KEYPAD_KEY_Q            0x15
#define ASUSDEC_KEYPAD_KEY_W            0x1D
#define ASUSDEC_KEYPAD_KEY_E            0x24
#define ASUSDEC_KEYPAD_KEY_R            0x2D
#define ASUSDEC_KEYPAD_KEY_T            0x2C
#define ASUSDEC_KEYPAD_KEY_Y            0x35
#define ASUSDEC_KEYPAD_KEY_U            0x3C
#define ASUSDEC_KEYPAD_KEY_I            0x43
#define ASUSDEC_KEYPAD_KEY_O            0x44
#define ASUSDEC_KEYPAD_KEY_P            0x4D
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE    0x54
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE   0x5B
#define ASUSDEC_KEYPAD_KEY_BACKSLASH    0x5D
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK     0x58
#define ASUSDEC_KEYPAD_KEY_A            0x1C
#define ASUSDEC_KEYPAD_KEY_S            0x1B
#define ASUSDEC_KEYPAD_KEY_D            0x23
#define ASUSDEC_KEYPAD_KEY_F            0x2B
#define ASUSDEC_KEYPAD_KEY_G            0x34
#define ASUSDEC_KEYPAD_KEY_H            0x33
#define ASUSDEC_KEYPAD_KEY_J            0x3B
#define ASUSDEC_KEYPAD_KEY_K            0x42
#define ASUSDEC_KEYPAD_KEY_L            0x4B
#define ASUSDEC_KEYPAD_KEY_SEMICOLON    0x4C
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE   0x52
#define ASUSDEC_KEYPAD_KEY_ENTER        0x5A
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT    0x12
#define ASUSDEC_KEYPAD_KEY_Z            0x1A
#define ASUSDEC_KEYPAD_KEY_X            0x22
#define ASUSDEC_KEYPAD_KEY_C            0x21
#define ASUSDEC_KEYPAD_KEY_V            0x2A
#define ASUSDEC_KEYPAD_KEY_B            0x32
#define ASUSDEC_KEYPAD_KEY_N            0x31
#define ASUSDEC_KEYPAD_KEY_M            0x3A
#define ASUSDEC_KEYPAD_KEY_COMMA        0x41
#define ASUSDEC_KEYPAD_KEY_DOT          0x49
#define ASUSDEC_KEYPAD_KEY_SLASH        0x4A
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0x59

#define ASUSDEC_KEYPAD_KEY_LEFT         0xE06B
#define ASUSDEC_KEYPAD_KEY_RIGHT        0xE074
#define ASUSDEC_KEYPAD_KEY_UP           0xE075
#define ASUSDEC_KEYPAD_KEY_DOWN         0xE072
#define ASUSDEC_KEYPAD_RIGHTWIN         0xE027
#define ASUSDEC_KEYPAD_LEFTCTRL         0x14
#define ASUSDEC_KEYPAD_LEFTWIN          0xE01F
#define ASUSDEC_KEYPAD_LEFTALT          0x11
#define ASUSDEC_KEYPAD_KEY_SPACE        0x29
#define ASUSDEC_KEYPAD_RIGHTALT         0xE011
#define ASUSDEC_KEYPAD_WINAPP           0xE02F
#define ASUSDEC_KEYPAD_RIGHTCTRL        0xE014
#define ASUSDEC_KEYPAD_HOME             0xE06C
#define ASUSDEC_KEYPAD_PAGEUP           0xE07D
#define ASUSDEC_KEYPAD_PAGEDOWN         0xE07A
#define ASUSDEC_KEYPAD_END              0xE069
/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU         0x5F                
#define ASUSDEC_YEN                     0x6A
#define ASUSDEC_MUHENKAN                0x67        
#define ASUSDEC_HENKAN                  0x64        
#define ASUSDEC_HIRAGANA_KATAKANA       0x13
#define ASUSDEC_RO                      0x51
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2                0x61
/********************************/

#endif

#define ASUSDEC_KEYPAD_LOCK             0xE071

#define ASUSDEC_KEYPAD_KEY_BREAK        0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND       0xE0

/////////////////////////////////////////////////////////////

#define CMD_KB_DISABLE		0x00
#define CMD_KB_ENABLE		0x01
#define CMD_LED_FLASH_START	0x02
#define CMD_LED_FLASH_END	0x03
#define CMD_CAPSKEY_LED_ON  0x04
#define CMD_CAPSKEY_LED_OFF 0x05

#define TEST_MODE		0x80
#define AMBER_ON		0x81
#define BLUE_ON			0x82
#define DISCHARGE		0x84
#define CHARGE			0x88
#define SHIP_MODE		0xa0

#define GSENSOR_IOCTL_MAGIC                     'a'
#define GSENSOR_IOCTL_KEYBOARD                  _IOW(GSENSOR_IOCTL_MAGIC, 0x11, int[2])

#define EC_IOCTL_MAGIC                          'e'
#define EC_IOCTL_KB_ENABLE                      GSENSOR_IOCTL_KEYBOARD
#define EC_IOCTL_TEST_MODE                      _IOW(EC_IOCTL_MAGIC, 0x01, int)
#define EC_IOCTL_FLASH_LED                      _IOW(EC_IOCTL_MAGIC, 0x02, int)
#define EC_IOCTL_AMBER_LED                      _IOW(EC_IOCTL_MAGIC, 0x03, int)
#define EC_IOCTL_BLUE_LED                       _IOW(EC_IOCTL_MAGIC, 0x04, int)
#define EC_IOCTL_FORCE_CHARGE_MODE              _IOW(EC_IOCTL_MAGIC, 0x05, int)
#define EC_IOCTL_SHIP_MODE                      _IOW(EC_IOCTL_MAGIC, 0x06, int)
#define EC_IOCTL_TP_ENABLE                      _IOW(EC_IOCTL_MAGIC, 0x07, int)
#define EC_IOCTL_TP_MODE                        _IOW(EC_IOCTL_MAGIC, 0x08, int)

#define EC_IOCTL_GET_KB_STATUS					_IOW(EC_IOCTL_MAGIC, 0x09, int)
#define EC_IOCTL_GET_TEST_MODE_STATUS			_IOW(EC_IOCTL_MAGIC, 0x0A, int)
#define EC_IOCTL_GET_FLASH_LED_STATUS			_IOW(EC_IOCTL_MAGIC, 0x0B, int)
#define KB_CAPSLOCK_TEST						_IOW(EC_IOCTL_MAGIC, 0x0E, int)
#define GET_CAPSLOCK_STATUS						_IOW(EC_IOCTL_MAGIC, 0x0F, int)

#define EC_IOCTL_ENABLE_TRACKING_ANGLE			_IOW(EC_IOCTL_MAGIC, 0x12, int)


struct blueberry_kb_data{
    int value;
    int input_keycode;
};

struct blueberry_gpio{
	int gpio;
	int active_low;
};

struct blueberry_kb {
	struct input_dev    *input_dev;
	struct i2c_client   *client;
    struct work_struct input_work;
    struct blueberry_gpio irq_io;
    struct blueberry_gpio arm_notify_ec_io;
    int irq;

#if defined(CONFIG_HAS_EARLYSUSPEND)
    	struct early_suspend    early_drv;
#endif
    u8 i2c_data[4];
	int value;
	int input_keycode;
	int flag_register;

	struct mutex operation_mutex;
	unsigned int kb_enable;		// 1:enable, 0:disable
	unsigned int kb_enable1;		// 1:enable, 0:disable
	unsigned int tp_enable;		// 1:enable, 0:disable
	unsigned int tp_mode;		// 1:abs, 0:rel
	unsigned int test_mode;		//
	unsigned int led_flash;		// 
	unsigned int amber_led_status;	// 
	unsigned int blue_led_status;	// 
	unsigned int bat_status;	// 1:force charging 2:force discharging
	unsigned int ship_mode;		// 
	unsigned int capskey_status;
	atomic_t debug_flag;
	unsigned int arm_suspend_status;
	unsigned int arm_early_suspend;

	struct workqueue_struct *keyboard_wq;

	unsigned int tracking_angle;

    //int ec_wakeup;          // 0 : ec shutdown when PAD in LP0, 1 : keep ec active when PAD in LP0,
    //int ap_wake_wakeup;     // 0 : no ap_wake wakeup signal, 1: get ap_wake wakeup signal
    //int ec_in_s3;           // 0: normal mode, 1: ec in deep sleep mode
};

#endif
