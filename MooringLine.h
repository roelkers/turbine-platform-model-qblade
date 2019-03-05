#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono/physics/ChMarker.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLoadContainer.h"

#include "PlatformParams.h"

class Monopile;

class MooringLine{
private:
  PlatformParams p ;
  chrono::fea::ChBuilderBeamANCF builder;
  chrono::ChVector<> mooringFairlead;
  chrono::ChVector<> mooringAnchor;

  std::shared_ptr<chrono::fea::ChBeamSectionCable> sectionCable;

  std::shared_ptr<chrono::fea::ChLinkPointFrame> constraintMooring;
  std::shared_ptr<chrono::fea::ChLinkPointFrame> constraintAnchor;

  double eModMooring;
  double mooringArea;
  double restLengthOfElement;

public:
  MooringLine(chrono::ChSystem& system, std::shared_ptr<chrono::fea::ChMesh> mesh, PlatformParams &p, double theta, std::shared_ptr<Monopile> monopile, std::shared_ptr<chrono::ChLoadContainer> loadContainer);
  void render();
  void setRestLengthAndPosition();
  double getTensionForce();
  std::shared_ptr<chrono::fea::ChLinkPointFrame> getConstraintMooring() const;
};

#endif
