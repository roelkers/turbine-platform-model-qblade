#include "PlatformParams.h"

using namespace chrono;

PlatformParams::PlatformParams(){

  towerSetupPos = ChVector<>(0,0,0);
  towerHeight = 200;
  towerRadius = 2.5;
  mass = 2.4764e6;
  massMomentInertiaInRollAndPitch = 1.2622e10;
  massMomentInertiaInYaw = 2.7879e6;
  monopileNrElements = 10;

  distanceZfromItoN = 68;
  distanceZfromWtoI = 10.6;
  distanceZfromGtoW = 52.7;
  distanceZfromBtoG = -0.9;
  distanceZfromEtoB = 72.6;

  //Initial rotation of the monopile
  qRotationX = Q_from_AngAxis(5 * CH_C_DEG_TO_RAD, VECT_X); //roll angle
  qRotationY= Q_from_AngAxis(5 * CH_C_DEG_TO_RAD, VECT_Y); //pitch angle
  qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z); //yaw angle
  //Translate to initial Position
  initPosVec = ChVector<>(0,0,-12.9);

  mooringLineNr = 0;
  mooringDiameter = 0.15;
  mooringStiffness = 84040;//8471133.49;
  mooringPreTensionForce = 890000;//case 1:89000;
  mooringRaleyghDamping = 0.3000;
  mooringNrElements = 3;
  mooringAnchorRadiusFromFairlead = 900;
  mooringPosFairleadZFromBottom = 80;
  mooringPosAnchorZ = -300;
  mooringDensity = 2431;

  //source: https:en.wikipedia.org/wiki/Drag_coefficient
  dragCoefficientCylinderAxial = 0.82;
  //source: http://sv.20file.org/up1/916_0.pdf
  dragCoefficientCylinderLateral = 1.20;

  seaLevel = 0; //sea level [m] in z-direction from origin
  rhoWater = 1000;
  g = 9.81;

  //visualisation params
  cSystemFactor = 50;
  dVectorFactor = 10;
}
