float cw=75;
float ch=100;
float cd=20;
String leftcom="COM3";
String rightcom="COM6";
float WIDTH=1920;
float HEIGHT=1080;

import processing.serial.*;
Serial leftPort;  
Serial rightPort;
//
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

//current flex
float[][] flex = new float[2][6];
//flex (last 3 readings)
float[][][] nflex = new float[2][6][3];
//flex ranges
float[][][] flexr = new float[2][6][3];
//flex constrained 
float[][][] flexc = new float[2][6][2];
//flex averages
float[][] flexavg = new float[2][6];

//current IMU readings
float[][][] imu = new float[2][6][6];
//IMU (last 3 readings)
float[][][] nimu = new float[2][6][18];
//IMU averages
float[][][] imuavg = new float[2][6][6];
//counter for storing past readings
//scaling for finger drawings
float[] fscale = {.8, .9, 1, .9, .8, 0};
//gyro offsets
float goff[][][] = new float[2][6][3];
//filtered gyro readings (last angle + gyro reading)
float gyro[][][] = new float[2][6][3];
//filtered angle orientation
float angles[][][] = new float[2][6][3];


void setup() {
  //size(1920,1080,P3D);
  fullScreen(P3D, 2);
  //fullScreen(P3D, SPAN);
  frameRate(180);
  background(0);
  noiseSeed(2);
  for (int k=0; k<2; k++) {
    for (int i=0; i<6; i++) {
      //averages
      nflex[k][i][0]=0;
      nflex[k][i][1]=0;
      nflex[k][i][2]=0;
      //range
      flexr[k][i][0]=0;
      flexr[k][i][1]=0;
      flexr[k][i][2]=0;
      //normalized flex 
      flexc[k][i][0]=0;
      flexc[k][i][1]=0;
      //current
      flex[k][i]=0;
      //averages
      for (int j=0; j<18; j++) {
        nimu[k][i][j]=0;
      }
      //current
      for (int j=0; j<3; j++) {
        imu[k][i][j]=0;
      }
    }

    for (int i=0; i<6; i++) {
      for (int j=0; j<3; j++) {
        goff[k][i][j]=0;
        gyro[k][i][j]=0;
        angles[k][i][j]=0;
      }
    }
  }

  leftPort = new Serial(this, leftcom, 115200);
  leftPort.clear();
  leftPort.buffer(84);

  //rightPort = new Serial(this, rightcom, 115200);
  //rightPort.clear();
  //rightPort.buffer(84);
}

float rot=.01;
float[] tx={0, 0};
float[] ty={0, 0};
float[] tz={0, 0};

void seriall() {
  serialEvent(leftPort, 0);
}
void serialr() {
  serialEvent(rightPort, 1);
}

void draw()
{
  //ortho();
  //serialEvent(leftPort, 0);
  //serialEvent(rightPort, 1);
  thread("seriall");
  // thread("serialr");
  for (int k=0; k<2; k++) {
    for (int i=0; i<6; i++) {
      flexavg[k][i]=(nflex[k][i][0]+nflex[k][i][1]+nflex[k][i][2])/3;
      //finding ranges of flex sensors
      if (flex[k][i]>flexr[k][i][0]) {
        flexr[k][i][0]=flex[k][i];
        flexr[k][i][2]=(flexr[k][i][0]-flexr[k][i][1]);
      }
      if (flex[k][i]<flexr[k][i][1]) {
        flexr[k][i][1]=flex[k][i];
        flexr[k][i][2]=(flexr[k][i][0]-flexr[k][i][1]);
      }
      flexc[k][i][0]=(flexavg[k][i]-flexr[k][i][1])/flexr[k][i][2];
      flexc[k][i][1]=1-flexc[k][i][0];
    }
    for (int i=0; i<6; i++) {
      for (int j=0; j<3; j++) {
        imuavg[k][i][j]=(nimu[k][i][j]+nimu[k][i][j+6]+nimu[k][i][j+12])/3;
      }
      for (int j=3; j<6; j++) {
        imuavg[k][i][j]=((nimu[k][i][j]+nimu[k][i][j+6]+nimu[k][i][j+12])/3);
        ;
      }
    }
  }
  ////printing average readings (last 3)
  //for (int i=0; i<6; i++) {
  //  println(i+1, nf(lflexavg[i], 0, 1), ",", "X:", nf(lavg[i][0], 2, 2), ",", "Y:", nf(lavg[i][1], 2, 2), ",", "Z:", nf(lavg[i][2], 2, 2), "GX:", nf(lavg[i][3], 2, 2), ",", "GY:", nf(lavg[i][4], 2, 2), ",", "GZ:", nf(lavg[i][5], 2, 2));
  //}
  //printing current readings
  for (int i=0; i<6; i++) {
    println(i+1, "RAW-F:", nf(flex[0][i], 0, 1), "-", nf(flexc[0][i][0], 0, 2), "%", ",", "X:", nf(imuavg[0][i][0], 2, 2), ",", "Y:", nf(imuavg[0][i][1], 2, 2), ",", "Z:", nf(imuavg[0][i][2], 2, 2), "GX:", nf(imuavg[0][i][3], 2, 2), ",", "GY:", nf(imuavg[0][i][4], 2, 2), ",", "GZ:", nf(imuavg[0][i][5], 2, 2));
  }
  //if (frameCount%5==0) {
  //  background(25);
  //}
  background(0);
  textSize(32);
  text(angles[0][5][0], 10, 50);
  text(angles[0][5][1], 10, 100);
  text(angles[0][5][2], 10, 150);

  text(roll[0][5], 10, 250);
  text(pitch[0][5], 10, 300);
  text(yaw[0][5], 10, 350);
  for (int kk=0; kk<1; kk++) {
    pushMatrix();
    translate((kk+1)*width/3, height/2, -2000);
    noFill();
    stroke(255);
    rotateX(PI/3);
    hand(kk);
    popMatrix();
  }
}

float[][] roll={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
float[][] pitch={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
float[][] yaw={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};

float dt=.00125;
float t=.02;
float a=t/(t+dt);
float a1=1-a;

float trans=.05;
float[][] rollcheck={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
float[][] pitchcheck={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
float[][] yawcheck={{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
void hand(int k) {
  tx[k]=tx[k]-imu[k][5][4]*trans;
  ty[k]=ty[k]+imu[k][5][3]*trans;
  tz[k]=tz[k]-imu[k][5][5]*trans;

  rollcheck[k][5]=roll[k][5];
  pitchcheck[k][5]=pitch[k][5];
  yawcheck[k][5]=yaw[k][5];

  roll[k][5]=atan2(imuavg[k][5][1], imuavg[k][5][2])*180/PI; 
  pitch[k][5]=atan2(-imuavg[k][5][0], sqrt((imuavg[k][5][1]*imuavg[k][5][1]) + (imuavg[k][5][2]*imuavg[k][5][2])))*180/PI;
  yaw[k][5]=atan2(sqrt((imuavg[k][5][0]*imuavg[k][5][0])+(imuavg[k][5][1]*imuavg[k][5][1])), imuavg[k][5][2]);


  //if (roll[k][5]>0 && rollcheck[k][5]<0) {
  //  rollcheck[k][5]=roll[k][5];
  //  roll[k][5]=-360-roll[k][5]);
  //} else if (roll[k][5]<0 && rollcheck[k][5]>0) {
  //  rollcheck[k][5]=roll[k][5];
  //  roll[k][5]=360+roll[k][5]);
  //}


  gyro[k][5][0]=angles[k][5][0]+imuavg[k][5][3]*dt;
  gyro[k][5][1]=angles[k][5][1]+imuavg[k][5][4]*dt;
  gyro[k][5][2]=angles[k][5][2]+imuavg[k][5][5]*dt;

  angles[k][5][0]=(gyro[k][5][0]*a)+(a1*roll[k][5]);
  angles[k][5][1]=(gyro[k][5][1]*a)+(a1*pitch[k][5]);
  angles[k][5][2]=(gyro[k][5][2]*a)+(a1*yaw[k][5]);
  //translate(ty[k], tz[k], tx[k]);
  translate(0, 300, 0);

  rotateX(angles[k][5][1]*PI/180);
  rotateY(angles[k][5][0]*PI/180);
  rotateZ(-angles[k][5][2]*PI/180);

  //rotz+=lavg[5][5];
  strokeWeight(5);
  quad(-250, -200, 250, -200, 200, 100, -200, 100);
  pushMatrix();
  translate(0, 100, 0);
  rotateX(-PI/5);
  quad(-200, 20, 200, 20, 150, 150, -150, 150);
  popMatrix();
  strokeWeight(1);
  //fill(0, 150, 0);
  fill(0, 100, 255);
  rotateZ(PI/2);
  accelv(k, 5);
  box(cw, ch, cd);
  rotateZ(-PI/2);
  thumb(k);
  for (int i=0; i<4; i++) {
    pushMatrix();
    strokeWeight(5);
    translate((mod[k]*-200+mod[k]*i*cw+mod[k]*50*i), -200, 0);
    sphere(30);
    rotateX(flexc[k][i][0]*PI/4);
    line(0, 0, 0, 0, fscale[i]*-300, 0);
    translate(0, fscale[i]*-300, 0);
    rotateX(flexc[k][i][0]*PI/2);
    line(0, 0, 0, 0, fscale[i]*-200, 0);
    translate(0, fscale[i]*-200, 0);
    rotateX(flexc[k][i][0]*PI/3);
    //rotateX(angles[k][i][0]*PI/180);
    //tilt(k, i);
    line(0, 0, 0, 0, fscale[i]*-100, 0);
    translate(0, fscale[i]*-100, 0);
    fill(0, 150, 0);

    rotateX(-flexc[k][i][0]*PI/3);
    rotateX(-flexc[k][i][0]*PI/2);
    rotateX(-flexc[k][i][0]*PI/4);


    rotateZ(angles[k][5][2]*PI/270);
    rotateY(-angles[k][5][0]*PI/180);
    rotateX(-angles[k][5][1]*PI/180);

    tilt(k, i);
    //accelv(k, i);
    strokeWeight(1);
    box(cw, ch, cd);
    popMatrix();
  }
}
int[] mod={1, -1};

void thumb(int k) {
  pushMatrix();
  translate(mod[k]*200, 200, -50);
  sphere(30);
  rotateZ(mod[k]*PI/2);
  rotateY(mod[k]*PI/3);
  rotateX(flexc[k][4][0]*PI/3);
  strokeWeight(5);
  line(0, 0, 0, 0, fscale[4]*-300, 0);
  translate(0, fscale[4]*-300, 0);
  rotateX(flexc[k][4][0]*PI/2);
  line(0, 0, 0, 0, fscale[4]*-250, 0);
  translate(0, fscale[4]*-250, 0);

  rotateX(-flexc[k][4][0]*PI/2);
  rotateX(-flexc[k][4][0]*PI/3);
  rotateY(-mod[k]*PI/3);
  rotateZ(-mod[k]*PI/2);

  rotateZ(angles[k][5][2]*PI/180);
  rotateY(-angles[k][5][0]*PI/180);
  rotateX(-angles[k][5][1]*PI/180);
  tilt(k, 4);
  //accelv(k, 4);

  strokeWeight(1);
  fill(255, 0, 0);
  box(cw, ch, cd);
  popMatrix();
}

void tilt(int k, int i) {
  //int zcheck=1;
  //float u=.1;
  //float roll2=0;
  //float pitch2=0;
  //if (lavg[i][2]>0) {
  //  zcheck=1;
  //} else {
  //  zcheck=-1;
  //}
  roll[k][i]=atan2(imuavg[k][i][0], imuavg[k][i][2])*180/PI;
  //pitch=atan2(-imuavg[k][i][1], (zcheck*sqrt((u*lavg[i][0]*lavg[i][0]) + (lavg[i][2]*lavg[i][2]))))*180/PI;
  yaw[k][i]=atan2(sqrt((imuavg[k][i][0]*imuavg[k][i][0])+(imuavg[k][i][1]*imuavg[k][i][1])), imuavg[k][i][2]);
  //roll2=atan2(lavg[i][0], sqrt((u*lavg[i][1]*lavg[i][1]) + (lavg[i][2]*lavg[i][2])))*180/PI; 
  pitch[k][i]=atan2(imuavg[k][i][1], (sqrt((imuavg[k][i][0]*imuavg[k][i][0]) + (imuavg[k][i][2]*imuavg[k][i][2]))))*180/PI;

  gyro[k][i][0]=angles[k][i][0]+imuavg[k][i][3]*dt;
  gyro[k][i][1]=angles[k][i][1]+imuavg[k][i][4]*dt;
  gyro[k][i][2]=angles[k][i][2]+imuavg[k][i][5]*dt;

  angles[k][i][0]=(gyro[k][i][0]*a)+(a1*pitch[k][i]);
  angles[k][i][1]=(gyro[k][i][1]*a)+(a1*roll[k][i]);
  angles[k][i][2]=(gyro[k][i][2]*a)+(a1*yaw[k][i]);

  rotateX(angles[k][i][0]*PI/180);
  rotateY(angles[k][i][1]*PI/180);



  //rotateX(pitch[k]*PI/180);
  //rotateY(-roll[k]*PI/180);
  //rotateZ(yaw*PI/180);
}
int vscale=200;
void accelv(int k, int i) {
  strokeWeight(4);
  stroke(255, 0, 0);
  line(0, 0, 0, imu[k][i][0]*vscale, 0, 0);
  stroke(0, 255, 0);
  line(imu[k][i][0]*vscale, 0, 0, imu[k][i][0]*vscale, -imu[k][i][1]*vscale, 0);
  stroke(0, 0, 255);
  line(imu[k][i][0]*vscale, -imu[k][i][1]*vscale, 0, imu[k][i][0]*vscale, -imu[k][i][1]*vscale, -imu[k][i][2]*vscale);
  //strokeWeight(6);
  stroke(255, 255, 0);
  line(0, 0, 0, imu[k][i][0]*vscale, -imu[k][i][1]*vscale, -imu[k][i][2]*vscale);
  stroke(255);
  strokeWeight(1);
}

//void index() {
//}

//void middle() {
//}

//void ring() {
//}

//void pinky() {
//}

String[] vals={" ", " "};
boolean[] firstContact = {false, false};
byte[][] inBuffer = new byte[2][84];
int negcheck=32767;
int[] clockcount={0, 0};
int[] off={0, 0};
void serialEvent(Serial port, int k) {
  if (firstContact[k] == false) {
    vals[k] = port.readStringUntil(10);
    if (vals[k]!=null) { 
      port.clear();          // clear the serial port buffer
      firstContact[k] = true;     // you've had first contact from the microcontroller
      port.write("A");       // ask for more
    }
  } else {
    if (port.available() > 83) {
      inBuffer[k]=port.readBytes(84);
      if (inBuffer[k] != null) {
        for (int readnum=0; readnum<6; readnum++) {
          flex[k][readnum]=int((inBuffer[k][0+(readnum*14)] << 8) | (inBuffer[k][1+(readnum*14)] & 0xff));
          imu[k][readnum][0]=int((inBuffer[k][2+(readnum*14)] << 8) | (inBuffer[k][3+(readnum*14)] & 0xff));
          imu[k][readnum][1]=int((inBuffer[k][4+(readnum*14)] << 8) | (inBuffer[k][5+(readnum*14)] & 0xff));
          imu[k][readnum][2]=int((inBuffer[k][6+(readnum*14)] << 8) | (inBuffer[k][7+(readnum*14)] & 0xff));
          imu[k][readnum][3]=int((inBuffer[k][8+(readnum*14)] << 8) | (inBuffer[k][9+(readnum*14)] & 0xff));
          imu[k][readnum][4]=int((inBuffer[k][10+(readnum*14)] << 8) | (inBuffer[k][11+(readnum*14)] & 0xff));
          imu[k][readnum][5]=int((inBuffer[k][12+(readnum*14)] << 8) | (inBuffer[k][13+(readnum*14)] & 0xff));

          for (int i=0; i<3; i++) {
            if (imu[k][readnum][i]>negcheck) {
              imu[k][readnum][i] = (-(imu[k][readnum][i] - negcheck)*ascale);
            } else {
              imu[k][readnum][i]=imu[k][readnum][i]*ascale;
            }
          }
          for (int i=3; i<6; i++) {
            if (imu[k][readnum][i]>negcheck) {
              imu[k][readnum][i] = (-(imu[k][readnum][i] - negcheck)*gscale);
            } else {
              imu[k][readnum][i]=imu[k][readnum][i]*gscale;
            }
          }
          nflex[k][readnum][off[k]]=flex[k][readnum];
          nimu[k][readnum][0+(off[k]*6)]=imu[k][readnum][0];
          nimu[k][readnum][1+(off[k]*6)]=imu[k][readnum][1];
          nimu[k][readnum][2+(off[k]*6)]=imu[k][readnum][2];
          nimu[k][readnum][3+(off[k]*6)]=imu[k][readnum][3];
          nimu[k][readnum][4+(off[k]*6)]=imu[k][readnum][4];
          nimu[k][readnum][5+(off[k]*6)]=imu[k][readnum][5];
        }
      }
      port.write("A");
      off[k]=off[k]+1;
      if (off[k]==3) {
        off[k]=0;
      }      
      clockcount[k]=clockcount[k]+1;
      redraw();
    }
  }
}
//  //////////////////////////////////////////////////////////
//  //RIGHT PORT
//  //////////////////////////////////////////////////////////
//  else //(port == rightPort) 
//  {
//    if (firstContactr == false) {
//      rvals = port.readStringUntil(10);
//      if (rvals!=null) { 
//        port.clear();          // clear the serial port buffer
//        firstContactr = true;     // you've had first contact from the microcontroller
//        port.write("A");       // ask for more
//      }
//    } else {
//      if (port.available() > 83) {
//        inBufferr=port.readBytes(84);
//        if (inBufferr != null) {
//          for (int readcountr=0; readcountr<6; readcountr++) {
//            rf[readcountr]=int((inBufferr[0+(readcountr*14)] << 8) | (inBufferr[1+(readcountr*14)] & 0xff));
//            rs[readcountr][0]=int((inBufferr[2+(readcountr*14)] << 8) | (inBufferr[3+(readcountr*14)] & 0xff));
//            rs[readcountr][1]=int((inBufferr[4+(readcountr*14)] << 8) | (inBufferr[5+(readcountr*14)] & 0xff));
//            rs[readcountr][2]=int((inBufferr[6+(readcountr*14)] << 8) | (inBufferr[7+(readcountr*14)] & 0xff));
//            rs[readcountr][3]=int((inBufferr[8+(readcountr*14)] << 8) | (inBufferr[9+(readcountr*14)] & 0xff));
//            rs[readcountr][4]=int((inBufferr[10+(readcountr*14)] << 8) | (inBufferr[11+(readcountr*14)] & 0xff));
//            rs[readcountr][5]=int((inBufferr[12+(readcountr*14)] << 8) | (inBufferr[13+(readcountr*14)] & 0xff));

//            for (int i=0; i<3; i++) {
//              if (rs[readcountr][i]>negcheck) {
//                rs[readcountr][i] = (-(rs[readcountr][i] - negcheck)*ascale);
//              } else {
//                rs[readcountr][i]=rs[readcountr][i]*ascale;
//              }
//            }
//            for (int i=3; i<6; i++) {
//              if (rs[readcountr][i]>negcheck) {
//                rs[readcountr][i] = (-(rs[readcountr][i] - negcheck)*gscale);
//              } else {
//                rs[readcountr][i]=rs[readcountr][i]*gscale;
//              }
//            }
//            rflex[readcountr][offl]=rf[readcountr];
//            raccels[readcountr][0+(offl*6)]=rs[readcountr][0];
//            raccels[readcountr][1+(offl*6)]=rs[readcountr][1];
//            raccels[readcountr][2+(offl*6)]=rs[readcountr][2];
//            raccels[readcountr][3+(offl*6)]=rs[readcountr][3];
//            raccels[readcountr][4+(offl*6)]=rs[readcountr][4];
//            raccels[readcountr][5+(offl*6)]=rs[readcountr][5];
//          }
//        }
//        port.write("A");
//        offr=offr+1;
//        if (offr==3) {
//          offr=0;
//        }      
//        clockcountr=clockcountr+1;
//        redraw();
//      }
//    }
//  }
//}
