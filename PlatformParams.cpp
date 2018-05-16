#include "PlatformParams.h"

using namespace chrono;

PlatformParams::PlatformParams(double towerHeight){

  this->towerHeight=135;
  towerRadius=5;
  towerDensity=138.63;
  towerSetupPos = ChVector<>(0, 0, 0);
  towerSetupDir = ChVector<>(0, 0, 1);

  //Initial rotation of the monopile
  qRotationX = Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_X);
  qRotationY= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Y);
  qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z);
  //Translate to initial Position
  initPosVec = ChVector<>(0,0,-100);

  mooringLineNr = 3;
  mooringDiameter = 0.15;
  mooringStiffness = 8471133.49;
  mooringPretension = 3*827652;
  mooringRaleyghDamping = 0.1000;
  mooringNrElements = 1;
  mooringAnchorRadiusFromFairlead = 600;
  mooringPosFairleadZInBodyCoords = -15;
  mooringPosBottomZ = -300;

  nacelleMass = 2.385e5;
  ballastMass = 7.71e5;

  seaLevel = 0; //sea level [m] in z-direction from origin
  rhoWater = 1000;
  g = 9.81;

  //visualisation params
  cSystemFactor = 50;
  dVectorFactor = 200;
}
