float cw=75;
float ch=100;
float cd=20;
String leftcom="COM3";
String rightcom="COM6";
float WIDTH=1920;
float HEIGHT=1080;
int negcheck=32767;

import processing.serial.*;
Serial leftPort;  
Serial rightPort;

PrintWriter output;

//Acceleration Limit  |   Sensitivity
//----------------------------------------
//2g                  |    16,384
//4g                  |    8,192
//8g                  |    4,096
//16g                 |    2,048
//
//Angular Velocity Limit  |   Sensitivity
//----------------------------------------
//250º/s                  |    131
//500º/s                  |    65.5
//1000º/s                 |    32.8
//2000º/s                 |    16.4
float ascale0 = .000061;
float ascale1 = .000122;
float ascale2 = .000244;
float ascale3 = .000488;
float[] ascales = {.000061, .000122, .000244, .000488};

float gscale0 = 0.007633;
float gscale1 = 0.015267;
float gscale2 = 0.030534;
float gscale3 = 0.061068;
float[] gscales = {0.007633, 0.015267, 0.015267, 0.061068};

float ascale=ascale2;
float gscale=gscale2;
Hand left = new Hand();
Hand right = new Hand();

void setup() {
  clear();
  delay(3000);
  //size(1000, 1000, P3D);
  fullScreen(P3D, 2);
  //fullScreen(P3D, SPAN);
  frameRate(60);
  background(0);
  noiseSeed(2);

  //Hand left = new Hand();
  output = createWriter("data.txt");
  //leftPort = new Serial(this, leftcom, 115200);
  //leftPort.clear();
  //leftPort.buffer(84);

  //rightPort = new Serial(this, rightcom, 115200);
  //rightPort.clear();
  //rightPort.buffer(84);
  //t0=millis();
}

float rot=.01;
float[] tx={0, 0};
float[] ty={0, 0};
float[] tz={0, 0};
int counter=0;
int instance=0;
void seriall() {
  serialEvent(leftPort, left);
}
void serialr() {
  serialEvent(rightPort, right);
}

void draw()
{
  //thread("seriall");
  // thread("serialr");
}

class Hand {
  byte[][] inBuffer = new byte[2][84];
  Boolean firstContact=false;
  String val="";
  PVector f1a, f2a, f3a, f4a, f5a;
  PVector f1g, f2g, f3g, f4g, f5g;
  PVector ba, bg;
  PVector[] ha = {f1a, f2a, f3a, f4a, f5a, ba};
  PVector[] hg = {f1g, f2g, f3g, f4g, f5g, bg};
  float flex[] ={0, 0, 0, 0, 0, 0};
  float[] result3= {0, 0, 0};
  float[] result5= {0, 0, 0, 0, 0};

  Hand() {
    //ba = new PVector(0, 0, 0);
    //bg = new PVector(0, 0, 0);
    for (int i=0; i<6; i++) {
      for (int j=0; j<3; j++) {
        ha[0] = new PVector(0, 0, 0);
        hg[0] = new PVector(0, 0, 0);
      }
    }
  }
  float[] getax() {
    for (int i=0; i<6; i++) {
      result5[i]=ha[i].x;
    }
    return result5;
  }
  float[] getay() {
    for (int i=0; i<6; i++) {
      result5[i]=ha[i].y;
    }
    return result5;
  }
  float[] getaz() {
    for (int i=0; i<6; i++) {
      result5[i]=ha[i].z;
    }
    return result5;
  }
  float[] getgx() {
    for (int i=0; i<6; i++) {
      result5[i]=hg[i].x;
    }
    return result5;
  }
  float[] getgy() {
    for (int i=0; i<6; i++) {
      result5[i]=hg[i].y;
    }
    return result5;
  }
  float[] getgz() {
    for (int i=0; i<6; i++) {
      result5[i]=hg[i].z;
    }
    return result5;
  }
  float[] getfa(int f) {
    result3[0]=ha[f].x;
    result3[1]=ha[f].y;
    result3[2]=ha[f].z;
    return result3;
  }
  float[] getfg(int f) {
    result3[0]=hg[f].x;
    result3[1]=hg[f].y;
    result3[2]=hg[f].z;
    return result3;
  }
}

void serialEvent(Serial port, Hand hand) {
  byte[] inBuffer = new byte[84];
  if (hand.firstContact == false) {
    hand.val = port.readStringUntil(10);
    if (hand.val!=null) { 
      port.clear();          // clear the serial port buffer
      hand.firstContact = true;     // you've had first contact from the microcontroller
      port.write("A");       // ask for more
    }
  } else {
    if (port.available() > 83) {
      inBuffer=port.readBytes(84);
      if (inBuffer != null) {
        //BINARY -> INT CONVERSION
        for (int readnum=0; readnum<6; readnum++) {
          hand.flex[readnum]=int((inBuffer[0+(readnum*14)] << 8) | (inBuffer[1+(readnum*14)] & 0xff));
          hand.ha[readnum].x=int((inBuffer[2+(readnum*14)] << 8) | (inBuffer[3+(readnum*14)] & 0xff));
          hand.ha[readnum].y=int((inBuffer[4+(readnum*14)] << 8) | (inBuffer[5+(readnum*14)] & 0xff));
          hand.ha[readnum].z=int((inBuffer[6+(readnum*14)] << 8) | (inBuffer[7+(readnum*14)] & 0xff));
          hand.hg[readnum].x=int((inBuffer[8+(readnum*14)] << 8) | (inBuffer[9+(readnum*14)] & 0xff));
          hand.hg[readnum].y=int((inBuffer[10+(readnum*14)] << 8) | (inBuffer[11+(readnum*14)] & 0xff));
          hand.hg[readnum].z=int((inBuffer[12+(readnum*14)] << 8) | (inBuffer[13+(readnum*14)] & 0xff));

          //ACCELERATIONS SIGN CONVERSION
          if (hand.ha[readnum].x>negcheck) {
            hand.ha[readnum].x = (-(hand.ha[readnum].x - negcheck)*ascale);
          } else {
            hand.ha[readnum].x=hand.ha[readnum].x*ascale;
          }
          if (hand.ha[readnum].y>negcheck) {
            hand.ha[readnum].y = (-(hand.ha[readnum].y - negcheck)*ascale);
          } else {
            hand.ha[readnum].y=hand.ha[readnum].y*ascale;
          }
          if (hand.ha[readnum].z>negcheck) {
            hand.ha[readnum].z = (-(hand.ha[readnum].z - negcheck)*ascale);
          } else {
            hand.ha[readnum].z=hand.ha[readnum].z*ascale;
          }
          //GYROSCOPES SIGN CONVERSION
          if (hand.hg[readnum].x>negcheck) {
            hand.hg[readnum].x = (-(hand.hg[readnum].x - negcheck)*gscale);
          } else {
            hand.hg[readnum].x=hand.hg[readnum].x*gscale;
          }
          if (hand.hg[readnum].y>negcheck) {
            hand.hg[readnum].y = (-(hand.hg[readnum].y - negcheck)*gscale);
          } else {
            hand.hg[readnum].y=hand.hg[readnum].y*gscale;
          }
          if (hand.hg[readnum].z>negcheck) {
            hand.hg[readnum].z = (-(hand.hg[readnum].z - negcheck)*gscale);
          } else {
            hand.hg[readnum].z=hand.hg[readnum].z*gscale;
          }
        }
      }
      port.write("A");
      redraw();
    }
  }
}
