
#ifndef vacuum_h
#define vacuum_h

void turnBrush(unsigned n); //600 disengaged //1100 engaged
void turnOffMotors();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void turnRandom();
void setSpeed(int rpm);
void enableSensors();
void sensorsCheck();
void startWorking();

#endif

