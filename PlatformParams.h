#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

class PlatformParams{

public:

  PlatformParams();

  double distanceGtoE;
  chrono::ChVector<> platformSetupPos;

  double platformMass;
  double platformHeight;
  double platformRadius;
  double platformMassMomentInertiaInRollAndPitch;
  double platformMassMomentInertiaInYaw;

  int platformNrElementsBelowTaper;
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
  double towerHeight;
  double towerCOGDistanceFromBottom;
  double towerRadiusTop;

  double nacelleMass;
  double nacelleDistanceDownstream;
  double nacelleDistanceToYawBearing;

  double hubMass;
  double hubDistanceUpstream;
  double hubDistanceToYawBearing;
  double hubDiameter;

  double bladeNr;
  double bladeMass;
  double bladeCOGDistanceFromRoot;

  int mooringLineNr;
  double mooringDiameter;
  double mooringStiffnessTimesLength;
  double mooringRaleyghDamping;
  double mooringUnstretchedLength;
  int mooringNrElements;
  double mooringRadiusToFairleadsFromCenter;
  double mooringLineWeightInWaterPerMeter;
  double mooringRadiusToAnchorFromCenter;
  double mooringPosFairleadZFromBottom;
  double mooringPosAnchorZ;
  double mooringDensity;

  double dragCoefficientCylinderAxial;
  double dragCoefficientCylinderLateral;

  double addedDampingX;
  double addedDampingY;
  double addedDampingZ;
  double addedDampingYaw;

  double seaLevel;
  double rhoWater;
  double g;

  //visualisation params
  double cSystemFactor;
  double dVectorFactor;
  double forceLineFactor;
};

#endif
