#ifndef PLATFORMPARAMS_H
#define PLATFORMPARAMS_H

#include "chrono/core/ChVector.h"

class PlatformParams{

public:

  PlatformParams(double towerHeight);

  double towerHeight;
  double towerRadius;
  double towerDensity;
  chrono::ChVector<> towerSetupPos;
  chrono::ChVector<> towerSetupDir;

  int mooringLineNr;
  double mooringDiameter;
  double mooringStiffness;
  double mooringPretension;
  double mooringRaleyghDamping;
  int mooringNrElements;
  double mooringAnchorRadiusFromFairlead;
  double mooringPosFairleadZInBodyCoords;
  double mooringPosBottomZ;

  double ballastMass;

  double seaLevel;
  double rhoWater;
  double g;
};

#endif
