#include "pca9685.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "PTDebug.h"
#include "PT_Misc/PT_MiscString.h"

#define MODE1 0x00			//Mode  register  1
#define MODE2 0x01			//Mode  register  2
#define SUBADR1 0x02		//I2C-bus subaddress 1
#define SUBADR2 0x03		//I2C-bus subaddress 2
#define SUBADR3 0x04		//I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock

PCA9685::PCA9685( int i2c_dev, int i2c_address ) :
    m_fd(-1),
    m_i2c_address( i2c_address ),
    m_bin_period(1)
{
    const std::string dev_name = "/dev/i2c-" + std::to_string(i2c_dev);
    PT_INFO("open " << dev_name );

    if ((m_fd = open(dev_name.c_str(), O_RDWR)) < 0) {
        PT_SEVERE("Couldn't open I2C Bus " << i2c_dev << ", openfd():open " <<  errno);
        m_fd = -1;
    }
    if ( m_fd > 0 && ioctl(m_fd, I2C_SLAVE, m_i2c_address) < 0) {
        PT_SEVERE("I2C slave " << i2c_address << " failed [openfd():ioctl " << errno);
    }

    if( m_fd < 0 )
        return;

    set_all_channels( 0, 0 );
    i2c_write( MODE2, 0x04 ); // totem pole
    i2c_write( MODE1, 0x01 ); // allcall
    setPWMFreq(50);

}

PCA9685::~PCA9685()
{
    if( m_fd > 0 ){
        i2c_write(MODE1, 0x10); //sleep
        close(m_fd);
    }
    m_fd = -1;
}

bool PCA9685::opened()
{
    return m_fd > 0;
}

int PCA9685::i2c_read(unsigned char addr)
{
    if (m_fd < 0) {
        PT_SEVERE("device not opened");
        return 0;
    }
    unsigned char buff[1];
    buff[0] = addr;
    if (write(m_fd, buff, 1) != 1) {
        PT_SEVERE( "i2c write addr failed " << errno);
        return 0;
    } else {
        if (read(m_fd, buff, 1) != 1) {
            PT_SEVERE("i2c read error " << errno );
        } else {
            return buff[0];
        }
    }
    return 0;
}

int PCA9685::i2c_write(unsigned char addr, unsigned char data)
{
    if (m_fd < 0) {
        PT_SEVERE("device not opened");
        return 0;
    }
    //PT_INFO("wr 0x" << p3t::to_string_hex((int)addr,2) << " = " << p3t::to_string_hex( (int)data,2) );
    unsigned char buff[2];
    buff[0] = addr;
    buff[1] = data;
    if (write(m_fd, buff, 2) != 2) {
        PT_SEVERE( "i2c write failed " << errno);
        return 0;
    }
    return 1;
}

int PCA9685::setPWMFreq(int freq)
{
    uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
    PT_INFO("set prescale to " << (int)prescale_val );
    i2c_write(MODE1, 0x10); //sleep
    i2c_write(PRE_SCALE, prescale_val); // multiplyer for PWM frequency
    i2c_write(MODE1, 0x80); //restart
    i2c_write(MODE2, 0x04); //totem pole (default)

    m_bin_period = (1.0 / (CLOCK_FREQ / (double)prescale_val / 4096.0)) / 4096.0;
    PT_INFO("bin period = " << m_bin_period*1000.*1000. << " us");

    return 0;
}

int PCA9685::setPWM(unsigned char channel, int value)
{
    //i2c_write(LED0_ON_L + LED_MULTIPLYER * (channel), on_value & 0xFF);
    //i2c_write(LED0_ON_H + LED_MULTIPLYER * (channel), on_value >> 8);
    i2c_write(LED0_OFF_L + LED_MULTIPLYER * (channel), value & 0xFF);
    i2c_write(LED0_OFF_H + LED_MULTIPLYER * (channel), value >> 8);
    return 0;
}

int PCA9685::set_all_channels(int on, int off)
{
    i2c_write(ALLLED_ON_L, on & 0xFF);
    i2c_write(ALLLED_ON_H, on >> 8);
    i2c_write(ALLLED_OFF_L, off & 0xFF);
    i2c_write(ALLLED_OFF_H, off >> 8);
    return 0;
}

double PCA9685::get_bin_period(){return m_bin_period;}
