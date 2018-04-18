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

class Buoyancy {
private:
  PlatformParams p;
  std::shared_ptr<chrono::ChBodyEasyCylinder> monopile;
  std::shared_ptr<chrono::ChLoadBodyForce> buoyancyForce;
  std::shared_ptr<chrono::ChLoadContainer> loadContainer;
  std::shared_ptr<chrono::ChMarker> markerBottom;
  std::shared_ptr<chrono::ChMarker> markerTop;
  chrono::ChVector<> buoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
  chrono::ChVector<> intersectionPoint;
  double maximumBuoyancyForce;

  //visualization & testing
  /*
  std::shared_ptr<ChBodyEasySphere> buoyancyCenterVizSphere;
  std::shared_ptr<ChBodyEasySphere> ipCenterVizSphere;
  std::shared_ptr<ChBodyEasySphere> bottomMarkerVizSphere;
  std::shared_ptr<ChBodyEasySphere> topMarkerVizSphere;
  */
public:
  Buoyancy(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<chrono::ChBodyEasyCylinder> monopile, std::shared_ptr<chrono::fea::ChMesh> mesh, chrono::ChSystem& system);
  void update();
  void computeBuoyancy(chrono::ChVector<> vecE, chrono::ChVector<> vecI);
  double computeMaximumBuoyancyForce();
  double computeBuoyancyForce(double submergedLength);

  std::shared_ptr<chrono::ChMarker> getMarkerTop(){return markerTop;}
  std::shared_ptr<chrono::ChMarker> getMarkerBottom(){return markerBottom;}
  chrono::ChVector<> getBuoyancyCenter(){return buoyancyCenter;}
  chrono::ChVector<> getIntersectionPoint(){return intersectionPoint;}
};

#endif
