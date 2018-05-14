#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono/physics/ChMarker.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"

#include "PlatformParams.h"

class MooringLine{
private:
  PlatformParams p ;
  chrono::fea::ChBuilderBeamANCF builder;
  chrono::ChVector<> mooringFairlead;
  chrono::ChVector<> mooringAnchor;
  //chrono::ChMarker monopileFairleadMarker;
public:
  MooringLine(chrono::ChSystem& system, std::shared_ptr<chrono::fea::ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<chrono::ChBody> monopile);
  void render();
  void setRestLengthAndPosition();
};

#endif
