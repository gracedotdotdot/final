#include"mbed.h"
#include "arm_math.h"
#include "FXOS8700CQ.h"
#include "bbcar.h"
#include <math.h>
#include <stdlib.h>
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "mbed_rpc.h"

#define bound 0.9

Thread thread1;

//Communication
Serial pc(USBTX,USBRX); //tx,rx
Serial uart(D1,D0); //tx,rx

//Servo
Ticker servo_ticker;
PwmOut pin9(D9), pin8(D8);
BBCar car(pin8, pin9, servo_ticker);
Ticker encoder_ticker;
DigitalIn pin3(D3);        // for encoder
DigitalInOut pin10(D10);   // for ultrasound
DigitalOut led_run(LED1);  // red led indicates run
DigitalOut led_turn(LED2); // blue led indicates turn
DigitalOut led_pic(LED3);
DigitalIn switch_start(SW2);


//accel for rotation pid
FXOS8700CQ acc(PTD9, PTD8, (0x1D<<1));
float state[3] = {0};
float Kp = 0, Ki = 0, Kd = 0;
float a0 = 0, a1 = 0, a2 = 0;
//The formula is:
//y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
//A0 = Kp + Ki + Kd
//A1 = (-Kp ) - (2 * Kd )
//A2 = Kd

//xbee functions & variables
void xbee_setup(void);
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
RawSerial xbee(D12, D11);
Thread t;
EventQueue queue;

// rpc functions & variables
I2C i2c( PTD9,PTD8);
void getData(Arguments *in, Reply *out);
RPCFunction rpcAcc(&getData, "getData");
char buf[256], outbuf[256];
int query_count=0;
Thread t_uart;

float ping_distance[3] = {0};

void pid_init(){

    state[0] = 0;

    state[1] = 0;

    state[2] = 0;

    a0 = Kp + Ki + Kd;

    a1 = (-Kp) - 2*Kd;

    a2 = Kd;

}
float pid_process(float in){

    int out = in*a0 + a1*state[0] + a2*state[1] + state[2];

    //update state

    state[1] = state[0];

    state[0] = in;

    state[2] = out;


    return out;

}
void pid_control(char rotation, float turn){
    //pid control setup

    Kp = 2.0; Ki = 1.0; Kd = 0;

    pid_init();


    //sensor setup

    acc.enable();

    SRAWDATA accel_data, magn_data;


    char buff[256];

    float degree, diff, target_degree;

    wait(1);

    acc.get_data(&accel_data, &magn_data);

    degree = atan2(magn_data.y, magn_data.x) * 180 / PI;


    if(rotation == 'l'){

        target_degree = degree - turn;

    }else if(rotation == 'r'){

        target_degree = degree + turn;

    }else{

        target_degree = degree;

    }


    if(target_degree < -180){

        target_degree = 360 + target_degree;

    }else if(target_degree > 180){

        target_degree = 360 - target_degree;

    }

    diff = degree - target_degree;


    //The car will continue to turn to the target degree until the error is small enough

    while(abs(diff) > 8){

        //Process the PID control

        float correction = pid_process(diff);

        //bound the value from -0.9 to -.9

        correction = car.clamp(correction, bound, -bound);

        float turn = (rotation == 'l') ? (1-abs(correction)) : (-1+abs(correction));

        car.turn(car.turn2speed(turn),turn);

        wait(0.1);

        acc.get_data(&accel_data, &magn_data);

        degree = atan2(magn_data.y, magn_data.x) * 180 / PI;


        diff = degree - target_degree;

        pc.printf("degree:%f, target: %f, diff:%f \r\n", degree, target_degree, diff);

    }
    car.stop();

    pid_init();


}
void recieve_thread(){

  char recv;
  xbee.printf("Mission 1: detect id img=");
  while(1){
    if(uart.readable()){
    //if(uart.getc()!='\n'){
      xbee.printf("read from openmv\r\n");
      recv = uart.getc();
      xbee.putc(recv);
    //}
    }
  }
  //  while(1) {

  //     if(uart.readable()){

  //           char recv = uart.getc();

  //           pc.putc(recv);

  //           pc.printf("\r\n");

  //     }

  //  }

}
void getData(Arguments *in, Reply *out){
  // //if(query_count<10){                 //print how many data collected since the last query
  // int query_count=0;
  //   xbee.printf("%d\r\n",num_data);
  //   pc.printf("query count= %d\r\n",query_count);
  //   pc.printf("xbee print %d\r\n",num_data);
  //   query_count++;
  // //}
    
}
void xbee_setup(){

  char xbee_reply[4];

  xbee.baud(9600);

  xbee.printf("+++");

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){

    pc.printf("enter AT mode.\r\n");

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

  }

  xbee.printf("ATMY 0x240\r\n");

  reply_messange(xbee_reply, "setting MY : 0x240");


  xbee.printf("ATDL 0x140\r\n");

  reply_messange(xbee_reply, "setting DL : 0x140");


  xbee.printf("ATID 0x1\r\n");

  reply_messange(xbee_reply, "setting PAN ID : 0x1");


  xbee.printf("ATWR\r\n");

  reply_messange(xbee_reply, "write config");


  xbee.printf("ATMY\r\n");

  check_addr(xbee_reply, "MY");


  xbee.printf("ATDL\r\n");

  check_addr(xbee_reply, "DL");


  xbee.printf("ATCN\r\n");

  reply_messange(xbee_reply, "exit AT mode");
}
void xbee_rx_interrupt(void)
{

  xbee.attach(NULL, Serial::RxIrq); // detach interrupt

  queue.call(&xbee_rx);

}
void xbee_rx(){

  while(xbee.readable()){

    memset(buf, 0, 256);      // clear buffer

    for(int i=0; i<255; i++) {

        char recv = xbee.getc();

        if ( recv == '\r' || recv == '\n' ) {

          xbee.printf("\r\n");

          break;

        }

        buf[i] = xbee.putc(recv);
              
    }

    //RPC::call(buf, outbuf);

    pc.printf("outbuf = %s\r\n", outbuf);

  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);

}
void reply_messange(char *xbee_reply, char *messange){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){

    pc.printf("%s\r\n", messange);

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

    xbee_reply[2] = '\0';

  }

}
void check_addr(char *xbee_reply, char *messenger){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  xbee_reply[3] = xbee.getc();

  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';

  xbee_reply[1] = '\0';

  xbee_reply[2] = '\0';

  xbee_reply[3] = '\0';

}
int main(){

  while(switch_start==1){
    car.stop();
    wait(0.5);
  } //wait for switch to turn on and start

  led_turn=0;
  led_pic=0;
  led_run=0;
  parallax_encoder encoder0(pin3, encoder_ticker);
  parallax_ping ping1(pin10);
  encoder0.reset();
  uart.baud(9600);
  xbee.printf("start!");
  t_uart.start(recieve_thread);


  /* Use ping to detect objects*/
  // Collect three distance data
  //pid_control('r',10);
  // xbee.printf("left\r\n");
  // car.turn(3,-0.1);
  // while(encoder0.get_cm()<2){}
  // encoder0.reset();
  // car.stop();
  // ping_distance[0]=(float)ping1;

  // xbee.printf("middle\r\n");
  // car.turn(3,-0.1);
  // while(encoder0.get_cm()<2){}
  // encoder0.reset();
  // car.stop();
  // ping_distance[1]=(float)ping1;

  // xbee.printf("right\r\n");
  // car.turn(3,-0.1);
  // while(encoder0.get_cm()<2){}
  // encoder0.reset();
  // car.stop();
  // ping_distance[2]=(float)ping1;
  

  // for(int i =0; i<3; i++){
  //   xbee.printf("ping_distance[%d]=%f\r\n", i, ping_distance[i]);
  // }
  // // Classify objects: 
  // if(ping_distance[0]<ping_distance[1] && ping_distance[1]<ping_distance[2]){
  //   xbee.printf("right triangle\r\n");
  // }else if(ping_distance[0]<ping_distance[1] && ping_distance[1]>ping_distance[2]){
  //   xbee.printf("double mountain\r\n");
  // }else if(ping_distance[0]>ping_distance[1] && ping_distance[1]<ping_distance[2]){
  //   xbee.printf("normal triangle\r\n");
  // }else{
  //   xbee.printf("square\r\n");
  // }
  

  // //Go straightforward 120 cm
  // //car.goStraight(100);
  // car.servo0.set_speed(110);
  // car.servo1.set_speed(-100);
  // xbee.printf("keep going!\r\n");
  // while(encoder0.get_cm()<110 ) {
  //   // if((float)ping1<10){
  //   //   xbee.printf("ping1<25");
  //   //   break;
  //   // }
  // };
  // encoder0.reset();
  // xbee.printf("finish!");
  // led_run=0;
  // car.stop();
  // wait(0.5);

  // // //scan qrcode for calibration

  // // //turn left 90 degree to enter mission 1
  // car.turn(55,-0.5);
  // xbee.printf("turn to mission1");
  // xbee.printf("turn left!\r\n");
  // while(encoder0.get_cm()<32) {};
  // encoder0.reset();
  // car.stop();
  // wait(1);
  // car.goStraight(100);
  // xbee.printf("enter mission1!\r\n");
  // while(encoder0.get_cm()<68) {
  //   // if((float)ping1<10) {
  //   //   xbee.printf("ping<10\r\n");
  //   //   break;
  //   // }
  // };
  // encoder0.reset();

  // //reverse parking
  // car.turn(-60,-0.3);
  // xbee.printf("turn to park!\r\n");
  // while(encoder0.get_cm()<25) {};
  // encoder0.reset();
  // car.stop();

  // car.goStraight(-55);
  // xbee.printf("parking!\r\n");
  // while(encoder0.get_cm()<15) {};
  // encoder0.reset();

  // //leave the park
  // car.goStraight(55);
  // xbee.printf("leaving the park!\r\n");
  // while(encoder0.get_cm()<15) {};
  // encoder0.reset();
  // car.stop();

  // //turn right to leave mission 1
  // car.turn(60, 0.3);
  // xbee.printf("turn right!\r\n");
  // while(encoder0.get_cm()<10) {};
  // encoder0.reset();
  // car.stop();

  // car.goStraight(55);
  // xbee.printf("leaving mission 1!\r\n");
  // while(encoder0.get_cm()<50) {};
  // encoder0.reset();
  // car.stop();

  // car.turn(55, 0.5);
  // xbee.printf("turn right!\r\n");
  // while(encoder0.get_cm()<20) {}; 
  // encoder0.reset();
  // car.stop();

  // //Go straightforward 120 cm
  // car.goStraight(100);
  // xbee.printf("keep going!\r\n");
  // while(encoder0.get_cm()<110) {};
  // encoder0.reset();
  // xbee.printf("finish!");
  // led_run=0;
  // car.stop();
  // wait(0.5);

  //send classify_img rpc to openmv cam
  //pid_control('r', 30);
  //take pic with openmv & classify
  int i=0;
  while(i<5){     //take 5 pictures
    char s[21];
    sprintf(s,"image_classification");
    uart.puts(s);
    xbee.printf("send img\r\n");
    i++;
    wait(1);
  }
  xbee.printf("end sending img signal\r\n");

  //upload img label to mqtt server




  



}
