#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

class PlatformParams{

public:

  PlatformParams();

  double distanceEtoG;
  chrono::ChVector<> towerSetupPos;

  double platformMass;
  double platformHeight;
  double platformRadius;
  double platformMassMomentInertiaInRollAndPitch;
  double platformMassMomentInertiaInYaw;

  int monopileNrElementsBelowTaper;
  double platformLengthBelowTaper;
  double platformLengthTaper;
  double platformLengthAboveTaper;

  double platformRadiusBelowTaper;
  double platformRadiusAboveTaper;

  chrono::ChQuaternion<> qRotationX;
  chrono::ChQuaternion<> qRotationY;
  chrono::ChQuaternion<> qRotationZ;
  chrono::ChVector<> initPosVec;
  chrono::ChVector<> initVelVec;
  chrono::ChVector<> initAngVelVec;

  int nrElementsTower;
  double towerMass;
  double nacelleMass;
  double hubMass;
  double bladeMass;

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
