#include "PlatformParams.h"

using namespace chrono;

PlatformParams::PlatformParams(double towerHeight){

  this->towerHeight=towerHeight;
  towerRadius=5;
  towerDensity=400;
  towerSetupPos = ChVector<>(0, 0, 0);
  towerSetupDir = ChVector<>(0, 0, 1);

  mooringLineNr = 3;
  mooringDiameter = 0.15;
  mooringYoungModulus = 200e9;
  mooringRaleyghDamping = 0.000;
  mooringNrElements = 10;
  mooringAnchorRadiusFromFairlead = 100;
  mooringPosFairleadZInBodyCoords = 0.5*towerHeight;
  mooringPosBottomZ = -100;
  mooringRestLengthRelative = 0.3;

  ballastMass = 0.5e6;

  seaLevel = 0;
  rhoWater = 1000;
  g = 9.81;
}
