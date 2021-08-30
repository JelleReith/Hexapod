float calcAngleSSS(float sideA, float sideB, float sideOpposing) {
  float angleRadians = acos( (sq(sideA) + sq(sideB) - sq(sideOpposing)) / (2 * sideA * sideB));
  float angleDegrees = degrees(angleRadians);
  return angleDegrees;
};

float calcSideSAS(float sideA, float angleA, float sideB) {
  float distance = sqrt(sq(sideA) + sq(sideB) - (2 * sideA * sideB * cos(radians(angleA))));
  return distance;
}

float calcAngleSSA(float sideA, float sideB, float angleA) { //z prox ang
  float angleRadians = (sideA * sin(radians(angleA)) / sideB);
  angleRadians = asin(angleRadians);
  float angleDegrees = degrees(angleRadians);
  angleDegrees = 180 - angleA - angleDegrees;
  return angleDegrees;
}



float calcDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
  float distance = sqrt(sq(x2 - x1) + sq(y2 - y1) + sq(z2 - z1));
  return distance;
}

float calcAngle(float x1, float y1, float z1, float x2, float y2, float z2) {
  float ab = (x1 * x2) + (y1 * y2) + (z1 * z2);
  float a = sqrt(sq(x1) + sq(y1) + sq(z1));
  float b = sqrt(sq(x2) + sq(y2) + sq(z2));
  float angleRadians = acos(ab / (a * b));
  float angleDegrees = degrees(angleRadians);
  return angleDegrees;
}

float calcCoordinateX(float x1, float y1, float x2, float y2, float dt) {
  float d = calcDistance(x1, y1, 0, x2, y2, 0);
  float t = dt / d;
  float x = ((1 - t) * x1 + t * x2);
  return x;
}

float calcCoordinateY(float x1, float y1, float x2, float y2, float dt) {
  float d = calcDistance(x1, y1, 0, x2, y2, 0);
  float t = dt / d;
  float y = ((1 - t) * y1 + t * y2);
  return y;
}
