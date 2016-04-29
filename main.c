/*
 * Author: Efraín Martínez
 *
 * This program controls a robotic hand using 2 operation modes: REMOTE, using a glove with
 * flex sensors on each finger, or VIRTUAL, using serial commands from bluetooth Android App
 * "Bluetooth SPP Pro".
 *
 * It also monitors the movement of the fingers and displays the information on LCD from Grove
 * Starter Kit and sends the readings on Intel Cloud Analytics site.
 *
 */

#include "mraa.hpp"

#include <iostream>
#include <unistd.h>
#include "mraa.h"
#include <stdio.h>
#include <math.h>
#include "grove.h"
#include "jhd1313m1.h"
#include <climits>
#include <sstream>
#include "UdpClient.hpp"
extern "C" {
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
}

#include "Intel_Edison_BT_SPP.hpp"

#define NODE "localhost"
#define SERVICE "41234"
#define COMP_NAME "temp"
/*
 * Control of 4 servos
 */
Intel_Edison_BT_SPP spp = Intel_Edison_BT_SPP();
UdpClient client;

upm::GroveButton* button = new upm::GroveButton(4);	// button connected to D4 (digital in)
upm::Jhd1313m1* lcd = new upm::Jhd1313m1(0);		// LCD connected to the default I2C bus


void temperature_update(int finger_position, upm::GroveButton* button,
		upm::Jhd1313m1 *lcd, int selector){
	// minimum and maximum movements registered, the initial values will be
	// replaced after the first read
	static int min_position = INT_MAX;
	static int max_position = INT_MIN;

	const int POSITION_RANGE_MIN_VAL = 600;
	const int POSITION_RANGE_MAX_VAL = 1200;

	float fade; // fade value [0.0 .. 1.0]
	uint8_t r, g, b; // resulting LCD backlight color components [0 .. 255]
	std::stringstream row_1, row_2; // LCD rows

	// update the min and max position values, reset them if the button is
	// being pushed
	if (button->value() == 1) {
		min_position = finger_position;
		max_position = finger_position;
	} else {
		if (finger_position < min_position) {
			min_position = finger_position;
		}
		if (finger_position > max_position) {
			max_position = finger_position;
		}
	}

	// display the position values on the LCD and the operation mode of robotic hand
	row_1 << "Pos1 " << finger_position << " Max" << max_position;
	if (selector){
		row_2 << "Op mode: Virtual"; // Using BT App
	}
	else{
		row_2 << "Op mode: Remote "; // Using glove
	}
	lcd->setCursor(0,0);
	lcd->write(row_1.str());
	lcd->setCursor(1,0);
	lcd->write(row_2.str());

	std::stringstream ss;
	ss << "{\"n\":\"" << COMP_NAME << "\",\"v\":" << finger_position << "}" << std::endl;
	client.writeData(ss);

	// set the fade value depending on where we are in the position range
	if (finger_position <= POSITION_RANGE_MIN_VAL) {
		fade = 0.0;
	} else if (finger_position >= POSITION_RANGE_MAX_VAL) {
		fade = 1.0;
	} else {
		fade = (float)(finger_position - POSITION_RANGE_MIN_VAL) /
				(POSITION_RANGE_MAX_VAL - POSITION_RANGE_MIN_VAL);
	}

	// fade the color components separately
	r = (int)(255 * fade);
	g = (int)(64 * fade);
	b = (int)(255 * (1 - fade));

	lcd->setColor(r, g, b);	// apply the calculated result
}

int main()
{
	// min a max values of flex sensors mapped to required values por PWM
	// PWM pulses must have a period of 20ms
	 int minServo1 = 500;
	 int minServo2 = 550;
	 int maxServo1 = 2300;
	 int maxServo2 = 1900;
	 double slope1 = double (maxServo1 - minServo1)/(596-1020);
	 double slope2 = double (maxServo2 - minServo2)/(609-1020);
	 double slope3 = double (maxServo1 - minServo1)/(596-1020);
	 double slope4 = double (maxServo1 - minServo1)/(593-1020);

	 // initialization of PWMs
	 mraa_init();
	 mraa_gpio_context d_pin2 = mraa_gpio_init(2);
	 mraa_gpio_dir(d_pin2, MRAA_GPIO_IN);
	 int mode_selector;

	 mraa_pwm_context pwm1 = mraa_pwm_init(3);
	 mraa_pwm_enable(pwm1,0 );
	 mraa_pwm_period_ms(pwm1,20);
	 mraa_pwm_enable(pwm1,1 );
	 mraa::Aio* a_pin1 = new mraa::Aio(1);

	 mraa_pwm_context pwm2 = mraa_pwm_init(5);
	 mraa_pwm_enable(pwm2,0 );
	 mraa_pwm_period_ms(pwm2,20);
	 mraa_pwm_enable(pwm2,1 );
	 mraa::Aio* a_pin2 = new mraa::Aio(2);

	 mraa_pwm_context pwm3 = mraa_pwm_init(6);
	 mraa_pwm_enable(pwm3,0 );
	 mraa_pwm_period_ms(pwm3,20);
	 mraa_pwm_enable(pwm3,1 );
	 mraa::Aio* a_pin3 = new mraa::Aio(3);

	 mraa_pwm_context pwm4 = mraa_pwm_init(9);
	 mraa_pwm_enable(pwm4,0 );
	 mraa_pwm_period_ms(pwm4,20);
	 mraa_pwm_enable(pwm4,1 );
	 mraa::Aio* a_pin4 = new mraa::Aio(0);

	// UdpClient class is wrapper for sending UDP data to iotkit-agent
	// UdpClient client;
	if (client.connectUdp(NODE, SERVICE) < 0) {
		std::cerr << "Connection to iotkit-agent failed, exiting" << std::endl;
		return mraa::ERROR_UNSPECIFIED;
	}

	// select onboard LED pin based on the platform type
	// create a GPIO object from MRAA using it
	mraa_platform_t platform = mraa_get_platform_type();
	mraa::Gpio* d_pin = NULL;
	switch (platform) {
		case MRAA_INTEL_GALILEO_GEN1:
			d_pin = new mraa::Gpio(3, true, true);
			break;
		case MRAA_INTEL_GALILEO_GEN2:
			d_pin = new mraa::Gpio(13, true, false);
			break;
		case MRAA_INTEL_EDISON_FAB_C:
			d_pin = new mraa::Gpio(13, true, false);
			break;
		default:
			std::cerr << "Unsupported platform, exiting" << std::endl;
			return MRAA_ERROR_INVALID_PLATFORM;
	}
	if (d_pin == NULL) {
		std::cerr << "Can't create mraa::Gpio object, exiting" << std::endl;
		return MRAA_ERROR_UNSPECIFIED;
	}

	// set the pin as output
	if (d_pin->dir(mraa::DIR_OUT) != MRAA_SUCCESS) {
		std::cerr << "Can't set digital pin as output, exiting" << std::endl;
		return MRAA_ERROR_UNSPECIFIED;
	}

	spp.open();		// Open BT SPP

	for (;;) {

		mode_selector = mraa_gpio_read(d_pin2);

		if(mode_selector){ // if mode_selector == 1, operates as Virtual mode
			temperature_update(a_pin1->read(), button, lcd, mode_selector);

			sleep(1);
			ssize_t size = spp.read();
			if (size > 0 && size < 32)
			{
				char * buf = spp.getBuf();
				if (buf[0] == 'o' && buf[1] == 'p'){ // Open hand sequence
					d_pin->write(1);
					mraa_pwm_pulsewidth_us (pwm1, maxServo1);
					mraa_pwm_pulsewidth_us (pwm2, maxServo2); // use maxServo2 only if using groove servo
					mraa_pwm_pulsewidth_us (pwm3, maxServo1);
					mraa_pwm_pulsewidth_us (pwm4, maxServo1);
					//mraa_pwm_pulsewidth_us (pwm5, maxServo1);
					std::cout << "value " << maxServo2 << std::endl;

					usleep(10000);
				}
				// close fingers
				if (buf[0] == 'c' && buf[1] == 'l'){ // Close hand sequence
					d_pin->write(0);
					mraa_pwm_pulsewidth_us (pwm1, minServo1);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					mraa_pwm_pulsewidth_us (pwm3, minServo1);
					mraa_pwm_pulsewidth_us (pwm4, minServo1);
					std::cout << "value " << minServo2 << std::endl;
					usleep(10000);
				}
				if (buf[0] == 's' && buf[1] == '1'){ // Closing fingers 1 by 1 sequence
					d_pin->write(0);
					mraa_pwm_pulsewidth_us (pwm3, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm4, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm1, minServo1);
					std::cout << "value " << minServo2 << std::endl;
					usleep(10000);
				}
				if (buf[0] == 's' && buf[1] == '2'){ // Come on sequence ;)
					d_pin->write(0);
					mraa_pwm_pulsewidth_us (pwm3, minServo1);
					mraa_pwm_pulsewidth_us (pwm4, minServo1);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					mraa_pwm_pulsewidth_us (pwm1, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, maxServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, maxServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, maxServo1);
					usleep(500000);
					mraa_pwm_pulsewidth_us (pwm2, minServo1);
					std::cout << "value " << minServo2 << std::endl;
					usleep(10000);
				}
			}
		}
		else{

		// adjust duty cycle of PWM according to mapped values of each flex sensor
		 int pin_value1 = a_pin1->read();
		 int pwidth1 = (minServo1 + slope1* (pin_value1-1000));
		 mraa_pwm_pulsewidth_us (pwm1, pwidth1);

		 int pin_value2 = a_pin2->read();
		 int pwidth2 = (minServo2 + slope2* (pin_value2-1014));
		 mraa_pwm_pulsewidth_us (pwm2, pwidth2);

		 int pin_value3 = a_pin3->read();
		 int pwidth3 = (minServo1 + slope3* (pin_value3-1000));
		 mraa_pwm_pulsewidth_us (pwm3, pwidth3);

		 int pin_value4 = a_pin4->read();
		 int pwidth4 = (minServo1 + slope4* (pin_value4-1000));
		 mraa_pwm_pulsewidth_us (pwm4, pwidth4);

		 temperature_update(pin_value1, button, lcd, mode_selector);

		 // Show numeric values of each sensor
		std::cout << "value " << pwidth1 << ", An1 " << pin_value1 << ", An2 " << pin_value2
				<< ", An3 " << pin_value3 << ", An4 " << pin_value4 << std::endl;
		usleep(100000);
		}
	}

	return MRAA_SUCCESS;
}

