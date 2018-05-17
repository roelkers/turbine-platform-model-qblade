#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

class PlatformParams{

public:

  PlatformParams(double towerHeight);

  double towerHeight;
  double towerRadius;
  double towerDensity;
  chrono::ChVector<> towerSetupPos;
  chrono::ChVector<> towerSetupDir;

  chrono::ChQuaternion<> qRotationX;
  chrono::ChQuaternion<> qRotationY;
  chrono::ChQuaternion<> qRotationZ;
  chrono::ChVector<> initPosVec;

  int mooringLineNr;
  double mooringDiameter;
  double mooringStiffness;
  double mooringPreTensionForce;
  double mooringRaleyghDamping;
  int mooringNrElements;
  double mooringAnchorRadiusFromFairlead;
  double mooringPosFairleadZInBodyCoords;
  double mooringPosBottomZ;

  double fairleadMass;
  double ballastMass;
  double nacelleMass;

  double seaLevel;
  double rhoWater;
  double g;

  //visualisation params
  double cSystemFactor;
  double dVectorFactor;
};

#endif
