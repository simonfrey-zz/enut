#ifndef PCA9685_H
#define PCA9685_H

#include <string>
#include <inttypes.h>
#include <array>

class PCA9685
{
public:
    PCA9685(int i2c_dev , int i2c_address);
    ~PCA9685();

    bool opened();

    int i2c_read( unsigned char addr );
    int i2c_write( unsigned char addr, unsigned char data );

    int setPWMFreq( int freq );
    int setPWM( unsigned char channel, int value );

    int set_all_channels( int on, int off );

    double get_bin_period();

    void pca_sleep();
    void pca_init();

private:
    int m_fd;
    int m_i2c_address;
    double m_bin_period;

};

#endif // PCA9685_H
