#ifndef BuoyancyForce_H
#define BuoyancyForce_H

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChNodeFEAxyz.h"

#include "PlatformParams.h"

class Monopile;

class Buoyancy {
private:
  PlatformParams p;
  std::shared_ptr<Monopile> monopile;
  std::shared_ptr<chrono::ChLoadBodyForce> buoyancyForce;
  std::shared_ptr<chrono::ChLoadContainer> loadContainer;
  chrono::ChVector<> buoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
  chrono::ChVector<> intersectionPoint;

public:
  Buoyancy(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile);
  void update();
  void computeBuoyancy(chrono::ChVector<> vecE, chrono::ChVector<> vecI);
  double computeBuoyancyForce(double submergedLength);
  void render();

};

#endif
