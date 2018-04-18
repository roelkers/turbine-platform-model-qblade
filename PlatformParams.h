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
  double mooringYoungModulus;
  double mooringRaleyghDamping;
  int mooringNrElements;
  double mooringL;
  double mooringPosFairleadZ;
  double mooringPosBottomZ;
  double mooringRestLength;

  double seaLevel;
  double rhoWater;
  double g;
};

#endif
