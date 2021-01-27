#include "mbed.h"
#include "AS5601.h"

AS5601::AS5601(PinName sda, PinName scl, int ppr_val, int offset_val) : i2c(sda,scl)
{
    i2c.frequency(400000);
    
    offset = offset_val;
    ppr = ppr_val;
}

void AS5601::init(){
    //初期化
    //平均化数設定
    int conf1_set = read(CONF1_REG);
    conf1_set |= SF_16X;
    write(CONF1_REG ,conf1_set);
    
    //PPR設定
    int abn_set = read(ABN);
    if(ppr < 0b1000)abn_set |= ppr;
    else            abn_set |= PPR2048;
    write(ABN,abn_set);
}


int AS5601::getAngleAbsolute()
{
    char cmd1=0x0E,cmd2=0x0F;
    int data=0;

    data=this->read(cmd1) << 8;
    data=data+this->read(cmd2);

    return data;
}

float AS5601::getAngleDegrees()
{
    return (((float)this->getAngleAbsolute()-offset) * 180) / 2048 ;
}


char AS5601::read(char address)
{
    char retval;
    i2c.write(I2C_ADDR * 2, &address, 1);
    i2c.read(I2C_ADDR * 2, &retval, 1);
    return retval;
}

int AS5601::write(char address, char data)
{
    char buf[2];
    buf[0] = address;
    buf[1] = data;
    int val = i2c.write(I2C_ADDR * 2, buf, 2);
    
    return val;
}