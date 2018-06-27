#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

class PlatformParams{

public:

  PlatformParams();

  chrono::ChVector<> towerSetupPos;

  double mass;
  double towerHeight;
  double towerRadius;
  double massMomentInertiaInRollAndPitch;
  double massMomentInertiaInYaw;
  int monopileNrElements;

  double distanceZfromItoN;
  double distanceZfromWtoI;
  double distanceZfromGtoW;
  double distanceZfromEtoG;

  chrono::ChQuaternion<> qRotationX;
  chrono::ChQuaternion<> qRotationY;
  chrono::ChQuaternion<> qRotationZ;
  chrono::ChVector<> initPosVec;
  chrono::ChVector<> initVelVec;
  chrono::ChVector<> initAngVelVec;

  int mooringLineNr;
  double mooringDiameter;
  double mooringStiffness;
  double mooringPreTensionForce;
  double mooringRaleyghDamping;
  int mooringNrElements;
  double mooringAnchorRadiusFromFairlead;
  double mooringPosFairleadZFromBottom;
  double mooringPosAnchorZ;
  double mooringDensity;

  double dragCoefficientCylinderAxial;
  double dragCoefficientCylinderLateral;

  double seaLevel;
  double rhoWater;
  double g;

  //visualisation params
  double cSystemFactor;
  double dVectorFactor;
  double forceLineFactor;
};

#endif
