#ifndef BuoyancyForce_H
#define BuoyancyForce_H

#include "chrono/physics/ChMarker.h"
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChSystem.h"
#include <chrono_fea/ChNodeFEAxyz.h>

#include "PlatformParams.h"

using namespace chrono;
using namespace chrono::fea;

class Buoyancy {
private:
  PlatformParams p;
  std::shared_ptr<ChBodyEasyCylinder> monopile;
  std::shared_ptr<ChLoadBodyForce> buoyancyForce;
  std::shared_ptr<ChLoadContainer> loadContainer;
  std::shared_ptr<ChMarker> markerBottom;
  std::shared_ptr<ChMarker> markerTop;
  ChVector<> buoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
  double maximumBuoyancyForce;

  //visualization & testing
  /*
  std::shared_ptr<ChBodyEasySphere> buoyancyCenterVizSphere;
  std::shared_ptr<ChBodyEasySphere> ipCenterVizSphere;
  std::shared_ptr<ChBodyEasySphere> bottomMarkerVizSphere;
  std::shared_ptr<ChBodyEasySphere> topMarkerVizSphere;
  */
public:
  Buoyancy(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile, std::shared_ptr<ChMesh> mesh, ChSystem& system);
  void update();
  void computeBuoyancy(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint);
  double computeMaximumBuoyancyForce();
  double computeBuoyancyForce(double submergedLength);
};

#endif
