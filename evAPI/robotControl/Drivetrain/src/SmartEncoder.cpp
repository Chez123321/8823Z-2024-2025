#include "../include/SmartEncoder.h"

SmartEncoder::SmartEncoder(vex::motor * motorEncoderObject) {  //constructor to use motor encoder
  motorEncoder = motorEncoderObject;
}

void SmartEncoder::setEncoderMotor(vex::motor * motorEncoderObject) {  //constructor to use motor encoder
  motorEncoder = motorEncoderObject;
}

void SmartEncoder::setEncoderRotation(vex::rotation * rotationEncoderObject) {  //constructor to use rotation sensor
  rotationEncoder = rotationEncoderObject;
}

void SmartEncoder::resetAll() {  //resets raw encoder and all offsets
  for(double& elem: encoderOffsets) {
    elem = 0;
  }
  encoderReset();
}

int SmartEncoder::newTracker() {  //adds a new septate tracker
  encoderOffsets.push_back(encoderRead());
  trackerCount++;
  return(trackerCount);
}

void SmartEncoder::resetTrackerPosition(int trackerID) {  //resets a specified tracker
  encoderOffsets[trackerID] = encoderRead();
}

double SmartEncoder::readTrackerPosition(int trackerID) {  //reads a tracker of a specified tracker
  return(encoderRead() - encoderOffsets[trackerID]);
}

double SmartEncoder::encoderRead() {  //reads the raw encoder, motor or rotation
  if(rotationEncoder != nullptr) {
    return(rotationEncoder->position(vex::rotationUnits::deg));
  } else {
    return(motorEncoder->position(vex::rotationUnits::deg));
  }
}

void SmartEncoder::encoderReset() {  //resets the encoder, motor or rotation
  if(rotationEncoder != nullptr) {
    rotationEncoder->resetPosition();
  } else {
    motorEncoder->resetPosition();
  }
}