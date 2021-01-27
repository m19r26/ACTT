#ifndef AS5601_H
#define AS5601_H

//I2Cアドレス(7bit)
#define I2C_ADDR 0x36

//レジスタアドレス
#define STATUE_REG 0x0B
#define CONF1_REG 0x07
#define CONF2_REG 0x08
#define ABN 0x09

//設定値
//SF value
#define SF_16X 0x00
#define SF_8X 0x01
#define SF_4X 0x10
#define SF_2X 0x11

//ABN value
#define PPR8 0b0000
#define PPR16 0b0001
#define PPR32 0b0010
#define PPR64 0b0011
#define PPR128 0b0100
#define PPR256 0b0101
#define PPR512 0b0110
#define PPR1024 0b0111
#define PPR2048 0b1000

class AS5601 {
    public:
        //クラス宣言
        AS5601(PinName sda, PinName scl, int ppr_val=PPR2048, int offset_val=0);
        
        void init();
        
        //出力値(補正有りdeg)
        float getAngleDegrees();
        
        //出力値(補正無し12bit)
        int getAngleAbsolute();
        
        //PPRを出力
        
    private:
        int write(char address, char data);
        char read(char address);
        I2C i2c;
        
        int offset,ppr;
    };

#endif