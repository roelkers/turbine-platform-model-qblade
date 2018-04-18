#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"

#include "PlatformParams.h"

using namespace chrono;
using namespace chrono::fea;

class MooringLine{
private:
  ChBuilderBeamANCF builder;
public:
  MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<ChBodyEasyCylinder> monopile);
  void render();
};

#endif
