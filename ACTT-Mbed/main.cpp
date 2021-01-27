#include "mbed.h"
#include "QEI.h"
#include "AS5601.h"
#include <stdlib.h>

Serial rpi(PA_9, PA_10);
PwmOut buzz(PB_1);
PwmOut servo(PA_6);
PwmOut pwm(PB_0);
BusOut motor(PA_4, PA_12);
DigitalOut motor_set(PA_3);

DigitalOut led(PA_8);
DigitalIn  pin[4]{PF_0, PF_1, PB_4, PB_5};
BusIn      RotarySW(PF_0, PF_1, PB_4, PB_5);

AS5601 as(D4, D5, PPR2048, 0);
QEI    qei(PA_1, PA_5, NC, 4096, QEI::X2_ENCODING);
Ticker control;

#define MODE_0 0xF
#define MODE_1 0xE
#define MODE_2 0xD
#define MODE_3 0xC
#define MODE_4 0xB
#define MODE_5 0xA
#define MODE_6 0x9
#define MODE_7 0x8
#define MODE_8 0x7
#define MODE_9 0x6

#define FREE    0x0
#define FORWARD 0x1
#define BACK    0x2
#define STOP    0x3

#define ENABLE  1
#define DISABLE 0

#define Kf   0.004166667
#define Kp   0.00333

#define TARGET_PULSE 30

int motor_joy = 0, servo_joy=0;
int flag=1, pre_flag=0;
int signal=0;
int enc=0, diff;

float power=0, angle=0, p;
float gain, rps;

double target=0;

const double sampling_time = 0.02;
const double move_per_pulse = 0.003094;


void led_test()
{
    led = 1;
    wait(0.5);
    led = 0;
    wait(0.5);
}

void motor_test()
{
    pwm.write(0.1);
    motor = BACK;
    wait(2);
    motor = STOP;
    wait(0.5);
    motor = FORWARD;
    wait(2);
    motor = STOP;
    wait(0.5);
}

void buzz_test()
{/*
    buzz.period(1.0/261.626);
    buzz.write(0.2);
    wait(0.05);
    buzz.write(0.0);
*/}

void servo_test()
{
    servo.pulsewidth(0.0017);  //Center potision
    wait(1);
    servo.pulsewidth(0.00145);  //Left potision :: max
    wait(1);
    servo.pulsewidth(0.00205);  //Right potision :: max
    wait(1);
}

void run_sample()
{
    if(power >= 0) {
        motor = FORWARD;
        p = power;
    } else if(power < 0) {
        motor = BACK;
        p = power * -1;
    }
    pwm.write(p + target);
    servo.pulsewidth(0.00175 + angle);
}

void control_handler()
{
    enc = qei.getPulses();
    
    led =! led;
    
    qei.reset();
}

void run()
{
    if(power >= 0) {
        motor = FORWARD;
        p = power;
    } else if(power < 0) {
        motor = BACK;
        p = power * -1;
    } else if(power == 0) {
        motor = STOP;
    }
    pwm.write(p/gain);
    servo.pulsewidth(0.0017 + angle);
}

void sub_joy()  // RasPiから得たコントローラデータを出力用に計算
{    
    if(rpi.getc() == 128) {
        motor_joy = rpi.getc();
        servo_joy = rpi.getc();
    }
    
    diff = enc * 3 - ((120 - motor_joy) / 2);
    
    power = Kf * (120 - motor_joy) / 1.9 - (Kp * diff);
    angle = (90.0 - servo_joy) / 257142.86;      // サーボの最大値と中央値の差分0.00035より算出
}

int main()
{   
    rpi.attach(&sub_joy, Serial::RxIrq);  //RasPiからのシリアル受信割り込み
    pwm.period_us(10000);  //100Hz
    servo.period(0.02);  //50Hz
    
    motor_set = ENABLE; 
    motor = FORWARD;
    
    led_test();
    for(int i=0;i<4;i++) pin[i].mode(PullUp);
    
    as.init();
    control.attach(&control_handler, sampling_time);
    
    while (1) {
        run();
        //servo_test();
        //run_sample();
        
        switch(RotarySW) {
            case MODE_0: gain = 0.6; break;
            case MODE_1: gain = 0.8; break;
            case MODE_2: gain = 1.0; break;
            case MODE_3: gain = 1.6; break;
            case MODE_4: gain = 2.0; break;
            case MODE_5: gain = 2.4; break;
            case MODE_6: gain = 2.8; break;
            case MODE_7: gain = 3.2; break;
            case MODE_8: gain = 3.6; break;
            case MODE_9: gain = 4.0; break;
            default: break;
        }
    }
}
