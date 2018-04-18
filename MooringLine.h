#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"

#include "PlatformParams.h"

class MooringLine{
private:
  chrono::fea::ChBuilderBeamANCF builder;
public:
  MooringLine(chrono::ChSystem& system, std::shared_ptr<chrono::fea::ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<chrono::ChBodyEasyCylinder> monopile);
  void render();
};

#endif
