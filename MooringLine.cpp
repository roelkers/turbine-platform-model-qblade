#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
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

MooringLine::MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<ChBody> monopile){

  std::shared_ptr<ChBeamSectionCable> sectionCable = std::make_shared<ChBeamSectionCable>();
  sectionCable->SetDiameter(p.mooringDiameter);
  sectionCable->SetYoungModulus(p.mooringYoungModulus);
  sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);

  //Starting Position of Mooring Line on the Monopile
  double xStart = p.towerRadius*sin(theta/180*M_PI);
  double yStart = p.towerRadius*cos(theta/180*M_PI);
  //End Position of Mooring Line
  double xEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*sin(theta/180*M_PI);
  double yEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*cos(theta/180*M_PI);

  ChVector<> mooringFairlead = ChVector<>(xStart, yStart, p.mooringPosFairleadZInBodyCoords);
  ChVector<> mooringAnchor = ChVector<>(xEnd, yEnd, p.mooringPosBottomZ);

  // Now, simply use BuildBeam to create a beam from a point to another:
  builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
    sectionCable,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
    p.mooringNrElements,      // the number of ChElementBeamANCF to create
    mooringFairlead,     // the 'A' point in space (beginning of beam)
    mooringAnchor        // the 'B' point in space (end of beam)
  );

  //determine mooring line length
  double mooringL = sqrt(pow(p.mooringPosBottomZ,2) + pow(p.towerRadius+p.mooringAnchorRadiusFromFairlead,2));
  qDebug()<< "mooringL: " << mooringL ;

  double sectionLength = mooringL/p.mooringNrElements;
  double mooringRestLength = sectionLength*p.mooringRestLengthRelative;

  qDebug() << "Rest Length: " << mooringRestLength << "\n";

  //Iterate over beam elements to set the rest length (length at rest position)
  std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
  for(auto &element : beamElements){
    //qDebug() << "Prev:RestLength: " << element->GetRestLength();
    element->SetRestLength(mooringRestLength);
    //qDebug() << "Now:RestLength: " << element->GetRestLength();
  }

  //create truss (fixed body)
  auto mtruss = std::make_shared<ChBody>();
  mtruss->SetBodyFixed(true);

  //fairleads constraint
  auto constraint_fairlead = std::make_shared<ChLinkPointFrame>();
  constraint_fairlead->Initialize(builder.GetLastBeamNodes().front(), monopile);
  //constraint_fairlead->SetAttachPositionInBodyCoords(mooringFairlead);
  //ChCoordsys<> mooringFairleadCoord = ChCoordsys<>(mooringFairlead);
  //constraint_fairlead->SetAttachReferenceInAbsoluteCoords(mooringFairleadCoord);
  system.Add(constraint_fairlead);

  //anchor constraint
  auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
  constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
  system.Add(constraint_hinge);

  //Create Markers for fairlead on monopile
  //To initialize the mooring lines after setup
  ChCoordsys<> fairleadCoordsys = ChCoordsys<>(mooringFairlead);
  monopileFairleadMarker.SetBody(monopile.get());
  monopileFairleadMarker.Impose_Abs_Coord(fairleadCoordsys);

  qDebug() << "monopileFairleadMarker x: " << monopileFairleadMarker.GetAbsCoord().pos.x();
  qDebug() << "monopileFairleadMarker y: " << monopileFairleadMarker.GetAbsCoord().pos.y();
  qDebug() << "monopileFairleadMarker z: " << monopileFairleadMarker.GetAbsCoord().pos.z();
}

void MooringLine::updateFairleadNode(){
    qDebug() << "updating fairlead node";
    monopileFairleadMarker.UpdateState();

    qDebug() << "monopileFairleadMarker x: " << monopileFairleadMarker.GetAbsCoord().pos.x();
    qDebug() << "monopileFairleadMarker y: " << monopileFairleadMarker.GetAbsCoord().pos.y();
    qDebug() << "monopileFairleadMarker z: " << monopileFairleadMarker.GetAbsCoord().pos.z();

    //update node position to where the marker of the monopile has moved during initialization
    std::shared_ptr<ChNodeFEAxyzD> fairleadNode = builder.GetLastBeamNodes().front();

    qDebug() << "fairleadNode before x: " << fairleadNode->GetPos().x();
    qDebug() << "fairleadNode before y: " << fairleadNode->GetPos().y();
    qDebug() << "fairleadNode before z: " << fairleadNode->GetPos().z();

    fairleadNode->SetPos(monopileFairleadMarker.GetAbsCoord().pos);

    qDebug() << "fairleadNode after x: " << fairleadNode->GetPos().x();
    qDebug() << "fairleadNode after y: " << fairleadNode->GetPos().y();
    qDebug() << "fairleadNode after z: " << fairleadNode->GetPos().z();
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
