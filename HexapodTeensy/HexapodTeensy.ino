#include <PWMServo.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHHardwareSPI2.h>
RH_NRF24 nrf24(38, 39, hardware_spi2);

typedef struct
{
  uint16_t xRadio;
  uint16_t yRadio;
  uint16_t zRadio;
  uint16_t pitchRadio;
  uint16_t rollRadio;
  uint16_t yawRadio;

} MyDataStruct;
MyDataStruct data;



PWMServo s0;
PWMServo s1;
PWMServo s2;

PWMServo s3;
PWMServo s4;
PWMServo s5;

PWMServo s6;
PWMServo s7;
PWMServo s8;

PWMServo s9;
PWMServo s10;
PWMServo s11;

PWMServo s12;
PWMServo s13;
PWMServo s14;

PWMServo s15;
PWMServo s16;
PWMServo s17;

int SERVOMAP[18] = { //
  2, 1, 0,
  15, 16, 17,
  12, 13, 14,
  9, 10, 11,
  8, 7, 6,
  3, 4, 5
};

int SERVOCAL[18][5] = {
  {750, 2250, 1625, 750, 1625}, //leg LF:  MIN,MAX,MIDDLE,180Deg,90Deg 0
  {750, 2250, 1430, 970, 1430}, //leg LF:  MIN,MAX,MIDDLE,135Deg,180Deg 1
  {770, 2250, 1530, 970, 2090},  //leg LF:  MIN,MAX,MIDDLE,-45Deg,45Deg 2

  {750, 2250, 1110, 2020, 1110}, //leg RF: MIN,MAX,MIDDLE,180Deg,90Deg 3
  {750, 2250, 1555, 1990, 1555}, //leg RF: MIN,MAX,MIDDLE,135Deg,180Deg 4
  {750, 2250, 1480, 920, 2040}, //leg RF: MIN,MAX,MIDDLE,-45Deg,45Deg 5

  {750, 2250, 1165, 2070, 1165}, //leg RM:  MIN,MAX,MIDDLE,180Deg,90Deg 6
  {750, 2250, 1570, 2005, 1570}, //leg RM:  MIN,MAX,MIDDLE,135Deg,180Deg 7
  {750, 2250, 1500, 940, 2060}, //leg RM:  MIN,MAX,MIDDLE,-45Deg,45Deg 8

  {750, 2250, 1170, 2070, 1170}, //leg RB:  MIN,MAX,MIDDLE,180Deg,90Deg 9
  {750, 2250, 1530, 1965, 1530}, //leg RB:  MIN,MAX,MIDDLE,135Deg,180Deg 10
  {750, 2250, 1520, 960, 2080}, //leg RB:  MIN,MAX,MIDDLE,-45Deg,45Deg 11

  {750, 2250, 1790, 915, 1790}, //leg LB:  MIN,MAX,MIDDLE,180Deg,90Deg 12
  {750, 2250, 1480, 1020, 1480}, //leg LB:  MIN,MAX,MIDDLE,135Deg,180Deg 13
  {750, 2250, 1500, 940, 2060}, //leg LB:  MIN,MAX,MIDDLE,-45Deg,45Deg 14

  {750, 2250, 1755, 850, 1755}, //leg LM:  MIN,MAX,MIDDLE,180Deg,90Deg 15
  {750, 2250, 1460, 1000, 1460}, //leg LM:  MIN,MAX,MIDDLE,135Deg,180Deg 16
  {750, 2250, 1510, 950, 2070}, //leg LM:  MIN,MAX,MIDDLE,-45Deg,45Deg 17
};

float femur = 27;
float tibia = 45;
float metatarsus = 75;

float legOriginOffset[6][3] = {
  { -31.5, 54.56, 0},
  {31.5, 54.56, 0},
  { 63, 0, 0},
  { 31.5, -54.56, 0},
  { -31.5, -54.56, 0},
  { -63, 0, 0}
}; //offset from center of body to rotate axis of leg

//float restingLegTipPositions[6][3] = { //leg, xyz offsets from origin
//  { -75, 131.428, 0},
//  { 75,  131.428, 0},
//  {150, 0, 0},
//  { 75, - 131.428, 0},
//  { -75, - 131.428, 0},
//  { -150, 0, 0},
//};

float restingLegTipPositions[6][3] = { //leg, xyz offsets from origin
  { -90, 131.428, 0},
  { 90,  131.428, 0},
  {150, 0, 0},
  { 90, - 131.428, 0},
  { -90, - 131.428, 0},
  { -150, 0, 0},
};

float legTip[6][3] = { //actual leg tip positions
  {restingLegTipPositions[0][0], restingLegTipPositions[0][1], restingLegTipPositions[0][2]},
  {restingLegTipPositions[1][0], restingLegTipPositions[1][1], restingLegTipPositions[1][2]},
  {restingLegTipPositions[2][0], restingLegTipPositions[2][1], restingLegTipPositions[2][2]},
  {restingLegTipPositions[3][0], restingLegTipPositions[3][1], restingLegTipPositions[3][2]},
  {restingLegTipPositions[4][0], restingLegTipPositions[4][1], restingLegTipPositions[4][2]},
  {restingLegTipPositions[5][0], restingLegTipPositions[5][1], restingLegTipPositions[5][2]},
};

float servoPosition[18] = {
  SERVOCAL[0][2], SERVOCAL[1][2], SERVOCAL[2][2],
  SERVOCAL[3][2], SERVOCAL[4][2], SERVOCAL[5][2],
  SERVOCAL[6][2], SERVOCAL[7][2], SERVOCAL[8][2],
  SERVOCAL[9][2], SERVOCAL[10][2], SERVOCAL[11][2],
  SERVOCAL[12][2], SERVOCAL[13][2], SERVOCAL[14][2],
  SERVOCAL[15][2], SERVOCAL[16][2], SERVOCAL[17][2]
};

float gaitPositions[6][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

float xB = 00;
float yB = 0.0;
float zB = 50;

float x_stick_max = 10;
float x_stick_min = -10;

float y_stick_max = 10;
float y_stick_min = -10;

float z_stick_max = 100;
float z_stick_min = 0;

float yaw_stick_max = 10;
float yaw_stick_min = -10;

float pitch_stick_max = 10;
float pitch_stick_min = -10;

float roll_stick_max = 10;
float roll_stick_min = -10;

void setServoPos(int theServo, float value) {
  servoPosition[SERVOMAP[theServo]] = value;
};


void moveServos() {
  s0.writeMicroseconds((servoPosition[0]));
  s1.writeMicroseconds((servoPosition[1]));
  s2.writeMicroseconds((servoPosition[2]));
  s3.writeMicroseconds((servoPosition[3]));
  s4.writeMicroseconds((servoPosition[4]));
  s5.writeMicroseconds((servoPosition[5]));
  s6.writeMicroseconds((servoPosition[6]));
  s7.writeMicroseconds((servoPosition[7]));
  s8.writeMicroseconds((servoPosition[8]));
  s9.writeMicroseconds((servoPosition[9]));
  s10.writeMicroseconds((servoPosition[10]));
  s11.writeMicroseconds((servoPosition[11]));
  s12.writeMicroseconds((servoPosition[12]));
  s13.writeMicroseconds((servoPosition[13]));
  s14.writeMicroseconds((servoPosition[14]));
  s15.writeMicroseconds((servoPosition[15]));
  s16.writeMicroseconds((servoPosition[16]));
  s17.writeMicroseconds((servoPosition[17]));
}

//========================================================================================================================================================================
void setup() {

  Serial.begin(115200);
  s0.attach(0, SERVOCAL[0][0], SERVOCAL[0][1]);
  s1.attach(1, SERVOCAL[1][0], SERVOCAL[1][1]);
  s2.attach(2, SERVOCAL[2][0], SERVOCAL[2][1]);

  s3.attach(3, SERVOCAL[3][0], SERVOCAL[3][1]);
  s4.attach(4, SERVOCAL[4][0], SERVOCAL[4][1]);
  s5.attach(5, SERVOCAL[5][0], SERVOCAL[5][1]);

  s6.attach(6, SERVOCAL[6][0], SERVOCAL[6][1]);
  s7.attach(7, SERVOCAL[7][0], SERVOCAL[7][1]);
  s8.attach(8, SERVOCAL[8][0], SERVOCAL[8][1]);

  s9.attach(9, SERVOCAL[9][0], SERVOCAL[9][1]);
  s10.attach(10, SERVOCAL[10][0], SERVOCAL[10][1]);
  s11.attach(11, SERVOCAL[11][0], SERVOCAL[11][1]);

  s12.attach(13, SERVOCAL[12][0], SERVOCAL[12][1]);
  s13.attach(14, SERVOCAL[13][0], SERVOCAL[13][1]);
  s14.attach(15, SERVOCAL[14][0], SERVOCAL[14][1]);

  s15.attach(19, SERVOCAL[15][0], SERVOCAL[15][1]);
  s16.attach(22, SERVOCAL[16][0], SERVOCAL[16][1]);
  s17.attach(23, SERVOCAL[17][0], SERVOCAL[17][1]);

  delay(10);

  //  while (!Serial)
  //    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(100))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");


  for (int i = 0; i < 18; i++) {
    setServoPos(i, SERVOCAL[i][2]);
  }
 // moveServos();

}



void rotateBody(float pitch, float roll, float yaw) {
  //rotateBody:: rotates body around xyz
  float cosa = cos(radians(yaw));
  float sina = sin(radians(yaw));
  float cosb = cos(radians(pitch));
  float sinb = sin(radians(pitch));
  float cosc = cos(radians(roll));
  float sinc = sin(radians(roll));
  float Axx = cosa * cosb;
  float Axy = cosa * sinb * sinc - sina * cosc;
  float Axz = cosa * sinb * cosc + sina * sinc;
  float Ayx = sina * cosb;
  float Ayy = sina * sinb * sinc + cosa * cosc;
  float Ayz = sina * sinb * cosc - cosa * sinc;
  float Azx = -sinb;
  float Azy = cosb * sinc;
  float Azz = cosb * cosc;
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    float px = legTip[theLeg][0];
    float py = legTip[theLeg][1];
    float pz = legTip[theLeg][2];
    legTip[theLeg][0] = Axx * px + Axy * py + Axz * pz;
    legTip[theLeg][1] = Ayx * px + Ayy * py + Ayz * pz;
    legTip[theLeg][2] = Azx * px + Azy * py + Azz * pz;
  }
};


void translateBody(float x, float y, float z) {
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    translateLeg(theLeg, x, y, z);
  }
};

void translateLeg(int theLeg, float x, float y, float z) {
  legTip[theLeg][0] = legTip[theLeg][0] + gaitPositions[theLeg][0] + x;
  legTip[theLeg][1] = legTip[theLeg][1] + gaitPositions[theLeg][1] + y;
  legTip[theLeg][2] = legTip[theLeg][2] + gaitPositions[theLeg][2] + z;
}

void moveBody(float x, float y, float z, float pitch, float roll, float yaw) {
  for (int i = 0; i < 6; i++) {
    for (int q = 0; q < 3; q++) {
      legTip[i][q] = restingLegTipPositions[i][q];
    }
  }
  rotateBody(pitch, roll, yaw);
  translateBody(x, y, z);
};

void calcLeg(int theLeg) {
  float legTipX;
  float legTipY;
  float legTipZ;
  legTipX = legTip[theLeg][0];
  legTipY = legTip[theLeg][1];
  legTipZ = legTip[theLeg][2];
  float legLength = calcDistance(legTipX, legTipY, legTipZ, legOriginOffset[theLeg][0],  legOriginOffset[theLeg][1], legOriginOffset[theLeg][2]);
  float proximalX = calcCoordinateX(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legTipX, legTipY, femur);
  float proximalY = calcCoordinateY(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legTipX, legTipY,  femur);
  float tipToProximalLength = calcSideSAS(legLength, calcAngle(legTipX, legTipY, legTipZ, proximalX, proximalY, 0), femur);
  float distalAngle = calcAngleSSS(metatarsus, tibia, tipToProximalLength);
  float realDistalAngle = map(distalAngle, 180, 90, SERVOCAL[0 + 3 * theLeg][3], SERVOCAL[0 + 3 * theLeg][4]);
  realDistalAngle = constrain(realDistalAngle, SERVOCAL[0 + 3 * theLeg][0], SERVOCAL[0 + 3 * theLeg][1]);
  setServoPos(0 + 3 * theLeg, (realDistalAngle));
  float proximalAngleTip = calcAngleSSS(tibia, tipToProximalLength, metatarsus);
  float legAngle = calcAngleSSA(legTipZ, tipToProximalLength, 90);
  float realProximalAngle = map(proximalAngleTip + legAngle + 90, 135, 180, SERVOCAL[1 + 3 * theLeg][3], SERVOCAL[1 + 3 * theLeg][4]);
  realProximalAngle =  constrain(realProximalAngle, SERVOCAL[1 + 3 * theLeg][0], SERVOCAL[1 + 3 * theLeg][1]);
  setServoPos(1 + 3 * theLeg, (realProximalAngle));
  float metacarpoAngleHelper[3] = {legOriginOffset[theLeg][0] + (legLength * cos(radians(30 + (theLeg * -60)))), legOriginOffset[theLeg][1] + (legLength * sin(radians(30 + (theLeg * -60)))), 0};
  float metacarpoAngle = calcAngleSSS(calcDistance(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legOriginOffset[theLeg][2], metacarpoAngleHelper[0], metacarpoAngleHelper[1], metacarpoAngleHelper[2]), calcDistance(legTipX, legTipY, 0, legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legOriginOffset[theLeg][2]), calcDistance(metacarpoAngleHelper[0], metacarpoAngleHelper[1], metacarpoAngleHelper[2], legTipX, legTipY, 0));
  metacarpoAngle = map(metacarpoAngle, 135, 45, -45, 45);
  float realMetacarpoAngle = map(metacarpoAngle, -45, 45, SERVOCAL[2 + 3 * theLeg][3], SERVOCAL[2 + 3 * theLeg][4]);
  realMetacarpoAngle =  constrain(realMetacarpoAngle, SERVOCAL[2 + 3 * theLeg][0], SERVOCAL[2 + 3 * theLeg][1]);
  setServoPos(2 + 3 * theLeg, (realMetacarpoAngle));
  //  Serial.println("translate leg");
  //  Serial.print("LegLength = \t");
  //  Serial.println(legLength);
  //  Serial.print("proximalX = \t");
  //  Serial.println(proximalX);
  //  Serial.print("proximalY = \t");
  //  Serial.println(proximalY);
  //  Serial.print("tip To Proximal length = \t");
  //  Serial.println(tipToProximalLength);
  //  Serial.print("proximalAngleTip = \t");
  //  Serial.println(proximalAngleTip);
  //  Serial.print("legAngle = \t");
  //  Serial.println(legAngle);
  //  Serial.print("metacarpoAngle = \t");
  //  Serial.println(metacarpoAngle);
}

void calcLegs() {//inverse kinematics for legs
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    calcLeg(theLeg);
  }
}

void translateGaitLeg(int theLeg, float x, float y, float z) {
  gaitPositions[theLeg][0] =  gaitPositions[theLeg][0] + x;
  gaitPositions[theLeg][1] =  gaitPositions[theLeg][1] + y;
  gaitPositions[theLeg][2] =  gaitPositions[theLeg][2] + z;
}

void translateGaitBody(float x, float y, float z) {
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    translateGaitLeg(theLeg, x, y, z);
  }
}

void rotateGaitLeg(int theLeg, float yaw) {
  float cosa = cos(radians(yaw));
  float sina = sin(radians(yaw));

  float cosb = cos(radians(0)); //not rotating around pitch
  float sinb = sin(radians(0)); //not rotating around pitch
  float cosc = cos(radians(0)); //not rotating around roll
  float sinc = sin(radians(0)); //not rotating around roll

  float Axx = cosa * cosb;
  float Axy = cosa * sinb * sinc - sina * cosc;
  float Axz = cosa * sinb * cosc + sina * sinc;
  float Ayx = sina * cosb;
  float Ayy = sina * sinb * sinc + cosa * cosc;
  float Ayz = sina * sinb * cosc - cosa * sinc;
  float Azx = -sinb;
  float Azy = cosb * sinc;
  float Azz = cosb * cosc;



  float px_old = legTip[theLeg][0];
  float py_old = legTip[theLeg][1];
  float pz_old = legTip[theLeg][2];
  float px_new = Axx * px_old + Axy * py_old + Axz * pz_old;
  float py_new = Ayx * px_old + Ayy * py_old + Ayz * pz_old;
  float pz_new = Azx * px_old + Azy * py_old + Azz * pz_old;

  float px_diff = px_new - px_old;
  float py_diff = py_new - py_old;
  float pz_diff = pz_new - pz_old;

  gaitPositions[theLeg][0] =  gaitPositions[theLeg][0] + px_diff;
  gaitPositions[theLeg][1] =  gaitPositions[theLeg][1] + py_diff;
  gaitPositions[theLeg][2] =  gaitPositions[theLeg][2] + pz_diff;

}

void rotateGaitBody(float yaw) {
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    rotateGaitLeg(theLeg, yaw);
  }
}

/*

  float cosa = cos(radians(yaw));
  float sina = sin(radians(yaw));
  float cosb = cos(radians(pitch));
  float sinb = sin(radians(pitch));
  float cosc = cos(radians(roll));
  float sinc = sin(radians(roll));
  float Axx = cosa * cosb;
  float Axy = cosa * sinb * sinc - sina * cosc;
  float Axz = cosa * sinb * cosc + sina * sinc;
  float Ayx = sina * cosb;
  float Ayy = sina * sinb * sinc + cosa * cosc;
  float Ayz = sina * sinb * cosc - cosa * sinc;
  float Azx = -sinb;
  float Azy = cosb * sinc;
  float Azz = cosb * cosc;
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    float px = legTip[theLeg][0];
    float py = legTip[theLeg][1];
    float pz = legTip[theLeg][2];
    legTip[theLeg][0] = Axx * px + Axy * py + Axz * pz;
    legTip[theLeg][1] = Ayx * px + Ayy * py + Ayz * pz;
    legTip[theLeg][2] = Azx * px + Azy * py + Azz * pz;
  }




*/

boolean firstSequence = true; //first part of sequence, if so, dont translate body because its already in correct position././
int gaitStep = 0; //the # gait step (from 0 to gaitStepNum)

float bodyXOffset;
float bodyYOffset;
float bodyYawOffset;
boolean reset_done = false;

void resetLegs() {
  reset_done = false;
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    boolean legCentered = false;
    if (gaitPositions[theLeg][0] > -1 && gaitPositions[theLeg][0] < 1) {
      gaitPositions[theLeg][0] = 0;
      if (gaitPositions[theLeg][1] > -1 && gaitPositions[theLeg][1] < 1) {
        gaitPositions[theLeg][1] = 0;
        legCentered = true;
      }
    }
    if (gaitPositions[theLeg][2] < 0) { //if leg is already in the air
      if (gaitPositions[theLeg][2] > -40 && !legCentered) { //if leg is not far enough in the air move it higher
        gaitPositions[theLeg][2] = gaitPositions[theLeg][2] - 1;
        return;
      }
      if (gaitPositions[theLeg][0] > 0) { //if leg x > 0 move it to center
        gaitPositions[theLeg][0] = gaitPositions[theLeg][0] - 1;
        return;
      }
      if (gaitPositions[theLeg][0] < 0) { //if leg x > 0 move it to center
        gaitPositions[theLeg][0] = gaitPositions[theLeg][0] + 1;
        return;
      }
      if (gaitPositions[theLeg][1] > 0) { //if leg x > 0 move it to center
        gaitPositions[theLeg][1] = gaitPositions[theLeg][1] - 1;
        return;
      }
      if (gaitPositions[theLeg][1] < 0) { //if leg x > 0 move it to center
        gaitPositions[theLeg][1] = gaitPositions[theLeg][1] + 1;
        return;
      }
      if (gaitPositions[theLeg][2] < 0) { //if leg x > 0 move it to center
        gaitPositions[theLeg][2] = gaitPositions[theLeg][2] + 1;
        return;
      }
    }
  }
  for (int theLeg = 0; theLeg < 6; theLeg++) {
    boolean legCentered = false;
    if (gaitPositions[theLeg][0] > -1 && gaitPositions[theLeg][0] < 1) {
      gaitPositions[theLeg][0] = 0;
      if (gaitPositions[theLeg][1] > -1 && gaitPositions[theLeg][1] < 1) {
        gaitPositions[theLeg][1] = 0;
        legCentered = true;
      }
    }
    if (gaitPositions[theLeg][2] > -40 && !legCentered) { //if leg is not far enough in the air move it higher
      gaitPositions[theLeg][2] = gaitPositions[theLeg][2] - 1;
      return;
    }
    if (gaitPositions[theLeg][0] > 0) { //if leg x > 0 move it to center
      gaitPositions[theLeg][0] = gaitPositions[theLeg][0] - 1;
      return;
    }
    if (gaitPositions[theLeg][0] < 0) { //if leg x > 0 move it to center
      gaitPositions[theLeg][0] = gaitPositions[theLeg][0] + 1;
      return;
    }
    if (gaitPositions[theLeg][1] > 0) { //if leg x > 0 move it to center
      gaitPositions[theLeg][1] = gaitPositions[theLeg][1] - 1;
      return;
    }
    if (gaitPositions[theLeg][1] < 0) { //if leg x > 0 move it to center
      gaitPositions[theLeg][1] = gaitPositions[theLeg][1] + 1;
      return;
    }
    if (gaitPositions[theLeg][2] < 0) { //if leg x > 0 move it to center
      gaitPositions[theLeg][2] = gaitPositions[theLeg][2] + 1;
      return;
    }
  }
  gaitStep = 0;
  firstSequence = true;
  reset_done = true;
}


int legHeight = 30; // stepheight of leg
float gaitStepNum = 600.0; //total number of gait steps
float stepDistance = 50.0; //distance the robot covers with one full step
float rotationAngle = 20; //how many degrees the robot turns with one full step
float sequenceLength = gaitStepNum / 4; //lenght of one sequence
float distancePerStep = stepDistance / gaitStepNum;
float xDistancePerStep;
float yDistancePerStep;
float yawAgnlePerStep;

void tripodGait(float stepSizeX, float stepSizeY, float stepSizeYaw) {
  xDistancePerStep = stepDistance * stepSizeX;
  yDistancePerStep = stepDistance * stepSizeY;
  yawAgnlePerStep = rotationAngle * stepSizeYaw;

  //  Serial.println(yDistancePerStep);
  //lift & forward 1 3 5
  //down 1 3 5
  //lift & forward 0 2 4
  //down 0 2 4



  if (gaitStep < 1 * sequenceLength) { //legs up & forward!
    rotateGaitLeg(1, -yawAgnlePerStep / gaitStepNum * 4);
    rotateGaitLeg(3, -yawAgnlePerStep / gaitStepNum * 4);
    rotateGaitLeg(5, -yawAgnlePerStep / gaitStepNum * 4);

    translateGaitLeg(1, 0, 0, -legHeight / sequenceLength);
    translateGaitLeg(3, 0, 0, -legHeight / sequenceLength);
    translateGaitLeg(5, 0, 0, -legHeight / sequenceLength);

    translateGaitLeg(1, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);
    translateGaitLeg(3, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);
    translateGaitLeg(5, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);

  } else if (gaitStep < 2 * sequenceLength) {//legs down
    translateGaitLeg(1, 0, 0, legHeight / sequenceLength );
    translateGaitLeg(3, 0, 0, legHeight / sequenceLength );
    translateGaitLeg(5, 0, 0, legHeight / sequenceLength );

  } else if (gaitStep < 3 * sequenceLength) {// legs up & forward
    rotateGaitLeg(0, -yawAgnlePerStep / gaitStepNum * 4);
    rotateGaitLeg(2, -yawAgnlePerStep / gaitStepNum * 4);
    rotateGaitLeg(4, -yawAgnlePerStep / gaitStepNum * 4);

    translateGaitLeg(0, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);
    translateGaitLeg(2, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);
    translateGaitLeg(4, -xDistancePerStep / gaitStepNum * 4, -yDistancePerStep / gaitStepNum * 4, 0);

    translateGaitLeg(0, 0, 0, -legHeight / sequenceLength );
    translateGaitLeg(2, 0, 0, -legHeight / sequenceLength );
    translateGaitLeg(4, 0, 0, -legHeight / sequenceLength );

  } else if (gaitStep < 4 * sequenceLength) {// legs down
    translateGaitLeg(0, 0, 0, legHeight / sequenceLength);
    translateGaitLeg(2, 0, 0, legHeight / sequenceLength);
    translateGaitLeg(4, 0, 0, legHeight / sequenceLength);
  }

  rotateGaitBody((yawAgnlePerStep / gaitStepNum));
  translateGaitBody((xDistancePerStep / gaitStepNum), (yDistancePerStep / gaitStepNum), 0);

  if (gaitStep > gaitStepNum) {
    gaitStep = 0;
    for (int theLeg = 0; theLeg < 6; theLeg ++) {
      for (int axis = 0; axis < 3; axis++) {
        gaitPositions[theLeg][axis] = 0;
      }
    }
  }
  gaitStep  = gaitStep + 1;
}

//void tripodGait(float stepSizeX, float stepSizeY, float stepSizeYaw) {
//  xDistancePerStep = stepDistance * stepSizeX;
//  yDistancePerStep = stepDistance * stepSizeY;
//  yawAgnlePerStep = rotationAngle * stepSizeYaw;
//  //  Serial.println(yDistancePerStep);
//  //lift 1 3 5
//  //forward 1 3 5
//  //down 1 3 5
//  //lift 0 2 4
//  //forward 0 2 4
//  //down 0 2 4
//  //
//
//
//  if (gaitStep < 1 * sequenceLength) { //legs up!
//    translateGaitLeg(1, 0, 0, -legHeight / sequenceLength);
//    translateGaitLeg(3, 0, 0, -legHeight / sequenceLength);
//    translateGaitLeg(5, 0, 0, -legHeight / sequenceLength);
//  } else if (gaitStep < 2 * sequenceLength) { //legs new pos
//    rotateGaitLeg(1, -yawAgnlePerStep / gaitStepNum * 6);
//    rotateGaitLeg(3, -yawAgnlePerStep / gaitStepNum * 6);
//    rotateGaitLeg(5, -yawAgnlePerStep / gaitStepNum * 6);
//    translateGaitLeg(1, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//    translateGaitLeg(3, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//    translateGaitLeg(5, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//  } else if (gaitStep < 3 * sequenceLength) {//legs down
//    translateGaitLeg(1, 0, 0, legHeight / sequenceLength );
//    translateGaitLeg(3, 0, 0, legHeight / sequenceLength );
//    translateGaitLeg(5, 0, 0, legHeight / sequenceLength );
//  } else if (gaitStep < 4 * sequenceLength) {// legs up
//    translateGaitLeg(0, 0, 0, -legHeight / sequenceLength );
//    translateGaitLeg(2, 0, 0, -legHeight / sequenceLength );
//    translateGaitLeg(4, 0, 0, -legHeight / sequenceLength );
//  } else if (gaitStep < 5 * sequenceLength) {//legs new pos
//    rotateGaitLeg(0, -yawAgnlePerStep / gaitStepNum * 6);
//    rotateGaitLeg(2, -yawAgnlePerStep / gaitStepNum * 6);
//    rotateGaitLeg(4, -yawAgnlePerStep / gaitStepNum * 6);
//    translateGaitLeg(0, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//    translateGaitLeg(2, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//    translateGaitLeg(4, -xDistancePerStep / gaitStepNum * 6, -yDistancePerStep / gaitStepNum * 6, 0);
//  } else if (gaitStep < 6 * sequenceLength) {// legs down
//    translateGaitLeg(0, 0, 0, legHeight / sequenceLength);
//    translateGaitLeg(2, 0, 0, legHeight / sequenceLength);
//    translateGaitLeg(4, 0, 0, legHeight / sequenceLength);
//  }
//
//  rotateGaitBody((yawAgnlePerStep / gaitStepNum));
//  translateGaitBody((xDistancePerStep / gaitStepNum), (yDistancePerStep / gaitStepNum), 0);
//
//  if (gaitStep > gaitStepNum) {
//    gaitStep = 0;
//    for (int theLeg = 0; theLeg < 6; theLeg ++) {
//      for (int axis = 0; axis < 3; axis++) {
//        gaitPositions[theLeg][axis] = 0;
//      }
//    }
//  }
//  gaitStep  = gaitStep + 1;
//}
//



float xL = 0.0;
float yL = 0.0;
float zL = 0.0;

float pitch = 0;
float roll = 0;
float yaw = 0;

long current_time;
long wait_time = 100;
long old_time;
uint8_t datalen = sizeof(data);


long current_time_gait;
long wait_time_gait = 100;
long old_time_gait;

int number_of_cycles_per_second;
int number_of_cycles_per_second_min = 1; //min walking speed
int number_of_cycles_per_second_max = 150; //max walking speed

boolean walking = false;
boolean resetting = false;


int cal_servo = 0;

int messageMillis = 0;
int lastMessageMillis = 0;

void loop() {
  //  if (Serial.available() > 0) {
  //    int value = Serial.parseInt();
  //
  //    if (value < 20) {
  //      cal_servo = value;
  //      Serial.println("new servo cal");
  //      Serial.println(cal_servo);
  //    } else {
  //      setServoPos(cal_servo, value);
  //      moveServos();
  //    }
  //  }
  //  if (false == true) {

  current_time = micros();
  current_time_gait = micros();

  if (current_time > old_time  + wait_time ) {
    if (nrf24.available()) {
      if (nrf24.recv((uint8_t*)&data, &datalen) && datalen == sizeof(data))
      {
        lastMessageMillis = messageMillis;
        messageMillis = millis();
        uint16_t xRadio = (data.xRadio);
        uint16_t yRadio = (data.yRadio);
        uint16_t zRadio = (data.zRadio);
        uint16_t pitchRadio = (data.pitchRadio);
        uint16_t rollRadio = (data.rollRadio);
        uint16_t yawRadio = (data.yawRadio);

        xB = map(xRadio, 0, 1024, x_stick_min * 10, x_stick_max * 10) / 10.0;
        yB = map(yRadio, 0, 1024, y_stick_min * 10, y_stick_max * 10) / 10.0;
        zB = map(zRadio, 0, 1024, z_stick_min * 10, z_stick_max * 10) / 10.0;
        pitch = map(pitchRadio, 0, 1024, pitch_stick_min * 10, pitch_stick_max * 10) / 10.0;
        roll = map(rollRadio, 0, 1024, roll_stick_min * 10,  roll_stick_max * 10) / 10.0;
        yaw = map(yawRadio, 0, 1024, yaw_stick_min * 10, yaw_stick_max * 10) / 10.0;
      }
      old_time = current_time;
    Serial.println(messageMillis - lastMessageMillis);
    Serial.println("-------------");
    }

  }

  
//  Serial.println(xB);
//  Serial.println(yB);
//  Serial.println(zB);
//  Serial.println(pitch);
//  Serial.println(roll);
//  Serial.println(yaw);

  
  
      // higher value: faster steps//
      // wait_time_fait is dependent on highest translation
  
      float stepSizeX = 0;
      float stepSizeY = 0;
      float stepSizeYaw = 0;
      if (reset_done) {
        if (xB > x_stick_max / 2 || xB < x_stick_min / 2 || yB > y_stick_max / 2 || yB < y_stick_min / 2 || yaw > yaw_stick_max / 2 || yaw < yaw_stick_min / 2) {
          if (current_time_gait > old_time_gait + wait_time_gait) {
            if (xB * xB > yB * yB &&  xB * xB > yaw * yaw ) { //xB bigger than yB
              number_of_cycles_per_second = sqrt(map(sq(xB), sq(x_stick_max / 2), sq(x_stick_max), sq(number_of_cycles_per_second_min), sq(number_of_cycles_per_second_max)));
              if (xB > 0) {
                stepSizeX = 1;
                stepSizeY = yB / xB;
                stepSizeYaw = yaw / xB;
              } else {
                stepSizeX = -1;
                stepSizeY = yB / -xB;
                stepSizeYaw = yaw / -xB;
              }
  
              // Serial.print(sqrt(number_of_cycles_per_second));
            } else if (yB * yB > yaw * yaw) {
              number_of_cycles_per_second = sqrt(map(sq(yB), sq(y_stick_max / 2), sq(y_stick_max), sq(number_of_cycles_per_second_min), sq(number_of_cycles_per_second_max)));
  
              if (yB > 0) {
                stepSizeX = xB / yB;
                stepSizeY = 1;
                stepSizeYaw = yaw / yB;
              } else {
                stepSizeX = xB / -yB;
                stepSizeY = -1;
                stepSizeYaw = yaw / -yB;
              }
  
              //Serial.print(sqrt(number_of_cycles_per_second));
            } else {
              number_of_cycles_per_second = sqrt(map(sq(yaw), sq(yaw_stick_max / 2), sq(yaw_stick_max), sq(number_of_cycles_per_second_min), sq(number_of_cycles_per_second_max)));
  
              if (yaw > 0) {
                stepSizeX = xB / yaw;
                stepSizeY = yB / yaw;
                stepSizeYaw = 1;
              } else {
                stepSizeX = xB / -yaw;
                stepSizeY = yB / -yaw;
                stepSizeYaw = -1;
              }
  
              //Serial.print(sqrt(number_of_cycles_per_second));
            }
  
            wait_time_gait = 100000 / number_of_cycles_per_second;
            //        Serial.print(stepSizeX);
            //        Serial.print("\t");
            //        Serial.print(stepSizeY);
            //        Serial.print("\t");
            //        Serial.print(stepSizeYaw);
            //        Serial.print("\t");
            //        Serial.println(wait_time_gait);
            tripodGait(stepSizeX, stepSizeY, stepSizeYaw);
            old_time_gait = current_time_gait;
          }
        }
        else {
          resetLegs();
        }
      }
      else {
        if (current_time_gait > old_time_gait + wait_time_gait) {
          wait_time_gait = 2500;
          resetLegs();
          old_time_gait = current_time_gait;
        }
      }
  
      moveBody(xB, yB, zB, pitch, roll, yaw);
      //moveBody(0, 0, zB, 0, 0, 0);
  
      calcLegs();
      moveServos();
      
}
