//
//  Serial.println("Check calcAngle");
////  float pointA[3] = {4, -3, 5};
////  float pointB[3] = {9, 7, -10};
//  float pointA[3] = {0, 10, 0};
//  float pointB[3] = {10, 0, 0};
//  Serial.print("point a = ");
//  Serial.print(pointA[0]);
//  Serial.print("\t");
//  Serial.print(pointA[1]);
//  Serial.print("\t");
//  Serial.println(pointA[2]);
//  
//  Serial.print("point b = ");
//  Serial.print(pointB[0]);
//  Serial.print("\t");
//  Serial.print(pointB[1]);
//  Serial.print("\t");
//  Serial.println(pointB[2]);
//
//  float angleA = calcAngle(pointA[0],pointA[1],pointA[2],pointB[0],pointB[1],pointB[2]);
//
//  Serial.print("angle =");
//  Serial.println(angleA);







//  Serial.println("Check calcCoordinate");
//  float legTip[3] = {66.174, 200, 0};
//  float legOrigin[3] = {66.174, 66.174, 0};
//  
//  Serial.print("legTip = ");
//  Serial.print(legTip[0]);
//  Serial.print("\t");
//  Serial.print(legTip[1]);
//  Serial.print("\t");
//  Serial.println(legTip[2]);
//  
//  Serial.print("legOrigin = ");
//  Serial.print(legOrigin[0]);
//  Serial.print("\t");
//  Serial.print(legOrigin[1]);
//  Serial.print("\t");
//  Serial.println(legOrigin[2]);
//
//  Serial.print("Proximal coordinates = \t");
//  Serial.print(calcCoordinateX(legOrigin[0],legOrigin[1],legTip[0],legTip[1],femur));
//  Serial.print("\t");
//  Serial.println(calcCoordinateY(legOrigin[0],legOrigin[1],legTip[0],legTip[1],femur));





//void translateLeg(int theLeg, float transX, float transY, float transZ) {//translateLeg:: moves tip of leg
//  // calculate legLength (distance from tip to origin of leg
//  // calculate coordinates of proximal interphalangeal
//  // calculate tipToProximal length (distance from tip to proximalinterphalangeal joint
//  // calculate distal interphalangeal angle
//  // calculate proximal interphalangeal angle
//  // calculate metacarpophalangeal angle
//
//  float legTipX = restingLegTipPositions[theLeg][0] + transX;
//  float legTipY = restingLegTipPositions[theLeg][1] + transY;
//  float legTipZ = restingLegTipPositions[theLeg][2] + transZ;
//
//
//  float legLength = calcDistance(legTipX, legTipY, legTipZ, legOriginOffset[theLeg][0],  legOriginOffset[theLeg][1], legOriginOffset[theLeg][2]);
//
//  float proximalX = calcCoordinateX(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legTipX, legTipY, femur);
//  float proximalY = calcCoordinateY(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legTipX, legTipY,  femur);
//
//  float tipToProximalLength = calcSideSAS(legLength, calcAngle(legTipX, legTipY, legTipZ, proximalX, proximalY, 0), femur);
//
//
//  float distalAngle = calcAngleSSS(metatarsus, tibia, tipToProximalLength);
//  float realDistalAngle = map(distalAngle, 180, 90, SERVOCAL[0 + 3 * theLeg][3], SERVOCAL[0 + 3 * theLeg][4]);
//  realDistalAngle = constrain(realDistalAngle, SERVOCAL[0 + 3 * theLeg][0], SERVOCAL[0 + 3 * theLeg][1]);
//  setServoPos(0 + 3 * theLeg, int(realDistalAngle));
//
//  float proximalAngleTip = calcAngleSSS(tibia, tipToProximalLength, metatarsus);
//
//
//  float legAngle = calcAngleSSA(transZ, tipToProximalLength, 90);
//
//
//  float realProximalAngle = map(proximalAngleTip + legAngle, 180, 90, SERVOCAL[1 + 3 * theLeg][3], SERVOCAL[1 + 3 * theLeg][4]);
//  realProximalAngle =  constrain(realProximalAngle, SERVOCAL[1 + 3 * theLeg][0], SERVOCAL[1 + 3 * theLeg][1]);
//  setServoPos(1 + 3 * theLeg, int(realProximalAngle));
//
//  float metacarpoAngle = calcAngleSSS(calcDistance(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legOriginOffset[theLeg][2], legOriginOffset[theLeg][0], legOriginOffset[theLeg][1] * 10, 0), calcDistance(legTipX, legTipY, 0, legOriginOffset[theLeg][0], legOriginOffset[theLeg][1], legOriginOffset[theLeg][2]), calcDistance(legOriginOffset[theLeg][0], legOriginOffset[theLeg][1] * 10, 0, legTipX, legTipY, 0));
//  metacarpoAngle = map(metacarpoAngle, 0, 90, 45, -45);
//
//
//  float realMetacarpoAngle = map(metacarpoAngle, -45, 45, SERVOCAL[2 + 3 * theLeg][3], SERVOCAL[2 + 3 * theLeg][4]);
//  realMetacarpoAngle =  constrain(realMetacarpoAngle, SERVOCAL[2 + 3 * theLeg][0], SERVOCAL[2 + 3 * theLeg][1]);
//  setServoPos(2 + 3 * theLeg, int(realMetacarpoAngle));
//
//  //  Serial.println("translate leg");
//  //  Serial.print("LegLength = \t");
//  //  Serial.println(legLength);
//  //  Serial.print("proximalX = \t");
//  //  Serial.println(proximalX);
//  //  Serial.print("proximalY = \t");
//  //  Serial.println(proximalY);
//  //  Serial.print("tip To Proximal length = \t");
//  //  Serial.println(tipToProximalLength);
//  //  Serial.print("proximalAngleTip = \t");
//  //  Serial.println(proximalAngleTip);
//  //  Serial.print("legAngle = \t");
//  //  Serial.println(legAngle);
//  //  Serial.print("metacarpoAngle = \t");
//  //  Serial.println(metacarpoAngle);
//}
