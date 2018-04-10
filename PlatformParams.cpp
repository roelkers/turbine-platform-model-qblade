#include "PlatformParams.h"

PlatformParams::PlatformParams(){

  towerHeight=100;
  towerRadius=5;
  towerDensity=600;
  towerInitPos = ChVector<>(0, 0, 0);
  towerInitDir = ChVector<>(0, 0, 1);

  mooringLineNr = 3;
  mooringDiameter = 0.15;
  mooringYoungModulus = 200e9;
  mooringRaleyghDamping = 0.000;
  mooringNrElements = 5;
  mooringL = 100;
  mooringPosFairleadZ = 0;
  mooringPosBottomZ = -100;
  double sectionLength = mooringL/mooringNrElements;
  //GetLog() << "sectionLength: " << sectionLength << "\n";
  mooringRestLength = sectionLength*0.3;

  seaLevel = 0;
  rhoWater = 1000;
  g = 9.81;
}
