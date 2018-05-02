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
  mooringStiffness = 8471133.49;
  mooringPretension = 2*827652;
  mooringRaleyghDamping = 0.000;
  mooringNrElements = 10;
  mooringAnchorRadiusFromFairlead = 100;
  mooringPosFairleadZInBodyCoords = 0.5*towerHeight;
  mooringPosBottomZ = -100;

  ballastMass = 2e6;

  seaLevel = 0;
  rhoWater = 1000;
  g = 9.81;
}
