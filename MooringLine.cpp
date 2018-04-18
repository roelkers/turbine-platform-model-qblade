#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "math.h"
#include "PlatformParams.h"
#include "MooringLine.h"

using namespace chrono;
using namespace chrono::fea;

MooringLine::MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<ChBodyEasyCylinder> monopile){

  std::shared_ptr<ChBeamSectionCable> sectionCable = std::make_shared<ChBeamSectionCable>();
  sectionCable->SetDiameter(p.mooringDiameter);
  sectionCable->SetYoungModulus(p.mooringYoungModulus);
  sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);
  // Shortcut!
  // This ChBuilderBeamANCF helper object is very useful because it will
  // subdivide 'beams' into sequences of finite elements of beam type, ex.
  // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF class.
  // If new nodes are needed, it will create them for you.

  //Starting Position of Mooring Line on the Monopile
  double xStart = p.towerRadius*sin(theta/180*M_PI);
  double yStart = p.towerRadius*cos(theta/180*M_PI);
  //End Position of Mooring Line
  double xEnd = (p.towerRadius+p.mooringL)*sin(theta/180*M_PI);
  double yEnd = (p.towerRadius+p.mooringL)*cos(theta/180*M_PI);

  // Now, simply use BuildBeam to create a beam from a point to another:
  builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
    sectionCable,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
    p.mooringNrElements,      // the number of ChElementBeamANCF to create
    ChVector<>(xStart, yStart, p.mooringPosFairleadZ),     // the 'A' point in space (beginning of beam)
    ChVector<>(xEnd, yEnd, p.mooringPosBottomZ)
  );  // the 'B' point in space (end of beam)

  //Iterate over beam elements to set the rest length (lenth at rest position)
  std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
  for(auto &element : beamElements){
    //GetLog() << "Prev:RestLength: " << element->GetRestLength() << "\n";
    element->SetRestLength(p.mooringRestLength);
    //GetLog() << "Now:RestLength: " << element->GetRestLength() << "\n";
  }

  // For instance, now retrieve the A end and add a constraint to
  // block the position only of that node:
  auto mtruss = std::make_shared<ChBody>();
  mtruss->SetBodyFixed(true);

  //fairleads constraint
  auto constraint_pos2 = std::make_shared<ChLinkPointFrame>();
  constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), monopile);
  system.Add(constraint_pos2);

  //anchor constraint
  auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
  constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
  system.Add(constraint_hinge);
}

void MooringLine::render(){
    //Iterate over nodes to visualize them
    glBegin(GL_LINE_STRIP);
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> beamNodes = builder.GetLastBeamNodes();
    for(auto &node : beamNodes){
        CVector nodePos = CVecFromChVec(node->GetPos());
        glVertex3d(nodePos.x,nodePos.y,nodePos.z);
    }
    glEnd();
}
