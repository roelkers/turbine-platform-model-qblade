#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

class PlatformParams{

public:

  PlatformParams();

  double distanceGtoE;

  double platformMass;
  double platformHeight;
  double platformRadius;
  double platformMassMomentInertiaInRollAndPitch;
  double platformMassMomentInertiaInYaw;

  int platformNrElementsBelowTaper;
  double platformLengthBelowTaper;
  double platformRadiusBelowTaper;

  double platformLengthTaper;
  int platformNrElementsTaper;

  double platformLengthAboveTaper;
  int platformNrElementsAboveTaper;
  double platformRadiusAboveTaper;

  chrono::ChVector<> initPosVec;
  chrono::ChVector<> initVelVec;
  chrono::ChVector<> initAngVelVec;

  chrono::ChVector<> initPosInterface;
  chrono::ChVector<> initRot;

  chrono::ChQuaternion<> initRotQuat;

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
  double mooringRadiusToAnchorFromCenter;
  double distanceZPlatformCOGtoFairlead;
  double mooringPosAnchorZ;
  double mooringDensityPerUnit;

  double dragCoefficientCylinderAxial;
  double dragCoefficientCylinderLateral;

  double addedDampingX;
  double addedDampingY;
  double addedDampingZ;
  double addedDampingYaw;
  double addedYawSpringStiffness;

  double addedMassCoefficient;

  double waveAmplitude;
  double wavePeriod;
  double waveLength;

  double rhoWater;
  double g;

  int nrRelaxations;
  int nrRelaxationSteps;

  //visualisation params
  double cSystemFactor;
  double dVectorFactor;
  double forceLineFactor;
};

#endif
