class Hand {
  byte[][] inBuffer = new byte[2][84];
  Boolean firstContact=false;
  String val="";
  PVector f1a, f2a, f3a, f4a, f5a;
  PVector f1g, f2g, f3g, f4g, f5g;
  PVector ba, bg;
  PVector[] ha = {f1a, f2a, f3a, f4a, f5a,ba};
  PVector[] hg = {f1g, f2g, f3g, f4g, f5g,bg};
  float flex[] ={0, 0, 0, 0, 0, 0};
  float[] result3= {0, 0, 0};
  float[] result5= {0, 0, 0, 0, 0};
  int count=0;
  float hist[][][] = new float[6][6][int(frameRate*2)];
  float fhist[][]= new float[6][int(frameRate*2)];

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








//class Hand {
//  float[][] imu = new float[6][6];
//  float [] flex = new float[6];
//  Hand() {
//    for (int i=0; i<6; i++) {
//      for (int j=0; j<6; j++) {
//        imu[i][j]=0;
//      }
//      flex[i]=0;
//    }
//  }
//  float getHandAccels() {
//  }
//}
