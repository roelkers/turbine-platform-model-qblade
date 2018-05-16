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
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

MooringLine::MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<Monopile> monopile)
:p(p)
{
  double mooringL = sqrt(pow(p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords,2) + pow(p.mooringAnchorRadiusFromFairlead,2));

  std::shared_ptr<ChBeamSectionCable> sectionCable = std::make_shared<ChBeamSectionCable>();
  sectionCable->SetDiameter(p.mooringDiameter);

  double mooringArea = M_PI*pow(p.mooringDiameter,2)/4;

  sectionCable->SetArea(mooringArea);

  double eModMooring = p.mooringStiffness*mooringL/mooringArea;

  sectionCable->SetYoungModulus(eModMooring);
  sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);

  //Starting Position of Mooring Line on the Monopile
  double xStart = p.towerRadius*sin(theta/180*M_PI);
  double yStart = p.towerRadius*cos(theta/180*M_PI);
  //End Position of Mooring Line
  double xEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*sin(theta/180*M_PI);
  double yEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*cos(theta/180*M_PI);

  double length_test = sqrt(pow((xEnd-xStart),2) + pow((yEnd-yStart),2) + pow((p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords),2));
  //double distance_test = sqrt(pow(xStart,2)+pow(yStart,2));

  qDebug() << "mooringLengthSetup :" << mooringL;
  qDebug() << "length mooring check:" << length_test;
  //qDebug() << "distance start to csystem origin:" << distance_test;

  //Coordinates of Fairlead in local frame of monopile, which is 90° turned around x axis
  //ChVector<> mooringFairlead = ChVector<>(xStart, yStart, p.mooringPosFairleadZInBodyCoords);
  mooringFairlead = monopile->getCylinder()->TransformPointLocalToParent(ChVector<>(xStart, p.mooringPosFairleadZInBodyCoords, -yStart));

  qDebug() << "created mooring fairlead";
  //Coordinates of anchor in parent coordinates, y with negative sign because of rotation of monopile?
  mooringAnchor = ChVector<>(xEnd, yEnd, p.mooringPosBottomZ);
  //ChVector<> mooringAnchor = monopile->TransformPointLocalToParent(ChVector<>(xEnd, p.mooringPosBottomZ, yEnd));

  // Now, simply use BuildBeam to create a beam from a point to another:
  builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
    sectionCable,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
    p.mooringNrElements,      // the number of ChElementBeamANCF to create
    mooringFairlead,     // the 'A' point in space (beginning of beam)
    mooringAnchor        // the 'B' point in space (end of beam)
  );

  /*
  double sectionLength = mooringL/p.mooringNrElements;
  double mooringRestLength = sectionLength*p.mooringRestLengthRelative;
  */

  //create truss (fixed body)
  auto mtruss = std::make_shared<ChBody>();
  mtruss->SetBodyFixed(true);

  //fairleads constraint
  auto constraint_fairlead = std::make_shared<ChLinkPointFrame>();
  constraint_fairlead->Initialize(builder.GetLastBeamNodes().front(), monopile->getCylinder());

  const ChVector<> pos = builder.GetLastBeamNodes().front()->GetPos();
  constraint_fairlead->SetAttachPositionInAbsoluteCoords(pos);

  //constraint_fairlead->SetAttachPositionInBodyCoords(mooringFairlead);
  //ChCoordsys<> mooringFairleadCoord = ChCoordsys<>(mooringFairlead);
  //constraint_fairlead->SetAttachReferenceInAbsoluteCoords(mooringFairleadCoord);
  system.Add(constraint_fairlead);

  //anchor constraint
  auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
  constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
  system.Add(constraint_hinge);

}

void MooringLine::setRestLengthAndPosition(){

    double mooringLengthSetup = sqrt(pow(p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords,2) + pow(p.mooringAnchorRadiusFromFairlead,2));
    qDebug()<< "mooringLengthSetup: " << mooringLengthSetup ;

    //Calculate actual length after initialization, this is necessary because the mooring lines
    //are not in the setup position (at the coordinate origin) anymore, so we need to account for that
    ChVector<> mooringActual = mooringAnchor - mooringFairlead;

    double mooringLengthInit = mooringActual.Length();
    qDebug()<< "mooringLengthInit: " << mooringLengthInit ;

    //thats the delta l for the setup position
    double deltal = p.mooringPretension/p.mooringStiffness;
    double frac = (mooringLengthSetup-deltal)/mooringLengthSetup;

    //factor for accounting initial position length
    double factorInit = mooringLengthSetup/mooringLengthInit;

    //Iterate over beam elements to set the rest length and rest position
    std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
    for(auto &element : beamElements){

        //qDebug() << "initial length"<<element->GetRestLength();

        double length = element->GetRestLength();
        element->SetRestLength(length*frac*factorInit);
        CVector pos = CVecFromChVec(element->GetNodeA()->GetX0());
        CVector pos0 = CVecFromChVec(element->GetNodeB()->GetX0());

        CVector lvec = CVector(pos-pos0);
        lvec.Normalize();
        lvec *= length*frac*factorInit;

        pos = pos0+lvec;
        ChVector<> chvec(pos.x,pos.y,pos.z);
        element->GetNodeB()->SetX0(chvec);

        //qDebug() << "new rest length"<<CVector(pos-pos0).VAbs() <<frac;
    }
}

void MooringLine::render(){
    glPointSize(0.1);
    glLineWidth(1);

    //Iterate over nodes to visualize them
    glBegin(GL_LINE_STRIP);
    glColor4d(0,0,0,1);
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> beamNodes = builder.GetLastBeamNodes();
    for(auto &node : beamNodes){
        CVector nodePos = CVecFromChVec(node->GetPos());
        glVertex3d(nodePos.x,nodePos.y,nodePos.z);
    }
    glEnd();
    //Render Coordinate system of fairlead
    glBegin(GL_LINES);
        ChVector<> fairleadPos = builder.GetLastBeamNodes().front()->GetPos();
        ChVector<> fairleadD = builder.GetLastBeamNodes().front()->GetD();
        CVector fairleadPosCVec = CVecFromChVec(fairleadPos);
        //red: x-axis
        glColor4d(0,1,0,1);
        glVertex3d(fairleadPosCVec.x,fairleadPosCVec.y,fairleadPosCVec.z);
        CVector dVectorEnd = CVecFromChVec(fairleadPos+p.dVectorFactor*fairleadD);
        glVertex3d(dVectorEnd.x,dVectorEnd.y,dVectorEnd.z);
        /*
        qDebug() << "fairleadPos x:" << fairleadPosCVec.x;
        qDebug() << "fairleadPos y:" << fairleadPosCVec.y;
        qDebug() << "fairleadPos z:" << fairleadPosCVec.z;

        qDebug() << "dVectorEnd x:" << dVectorEnd.x;
        qDebug() << "dVectorEnd y:" << dVectorEnd.y;
        qDebug() << "dVectorEnd z:" << dVectorEnd.z;
        */
    glEnd();
}
