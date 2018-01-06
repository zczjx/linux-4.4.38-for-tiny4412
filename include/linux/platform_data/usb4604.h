#ifndef __USB4604_H__
#define __USB4604_H__

#define USB4604_NAME	"usb4604"


enum usb4604_mode {
	USB4604_MODE_UNKNOWN = 1,
	USB4604_MODE_HUB,
	USB4604_MODE_STANDBY,
};

struct usb4604_platform_data {
	enum usb4604_mode	initial_mode;
	int	gpio_reset;
};

#endif
