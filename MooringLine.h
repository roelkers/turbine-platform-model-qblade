#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono/physics/ChMarker.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLinkMate.h"

#include "PlatformParams.h"

class Monopile;

class MooringLine{
private:
  PlatformParams p ;
  chrono::fea::ChBuilderBeamANCF builder;
  chrono::ChVector<> mooringFairlead;
  chrono::ChVector<> mooringAnchor;

  std::shared_ptr<chrono::ChBody> fairleadBody;
  std::shared_ptr<chrono::fea::ChLinkPointFrame> constraintMooring;
  std::shared_ptr<chrono::fea::ChLinkPointFrame> constraintAnchor;
  std::shared_ptr<chrono::ChLinkMateFix> constraintFairlead;

  double eModMooring;
  double mooringArea;
  double mooringLengthSetup;

public:
  MooringLine(chrono::ChSystem& system, std::shared_ptr<chrono::fea::ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<Monopile> monopile);
  void render();
  void setRestLengthAndPosition();
  double getTensionForceAt(double pos);
};

#endif
