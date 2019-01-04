#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono_fea/ChLoadsBeam.h"

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

std::shared_ptr<chrono::fea::ChLinkPointFrame> MooringLine::getConstraintMooring() const
{
    return constraintMooring;
}

MooringLine::MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, PlatformParams p, double theta, std::shared_ptr<Monopile> monopile, std::shared_ptr<ChLoadContainer> loadContainer)
:p(p)
{

  sectionCable = std::make_shared<ChBeamSectionCable>();
  sectionCable->SetDiameter(p.mooringDiameter);
  qDebug() << "mooring diameter" << p.mooringDiameter;

  mooringArea = M_PI*pow(p.mooringDiameter,2)/4;

  sectionCable->SetArea(mooringArea);
  qDebug() << "mooring Area" << mooringArea;

  eModMooring = p.mooringStiffnessTimesLength/mooringArea;

  qDebug() << "eModMooring" << eModMooring;

  sectionCable->SetYoungModulus(eModMooring);
  sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);

  sectionCable->SetDensity(p.mooringDensityPerUnit/mooringArea);

  qDebug() << "mooring density: " << p.mooringDensityPerUnit/mooringArea;

  //Starting Position of Mooring Line on the Monopile
  double xStart = p.mooringRadiusToFairleadsFromCenter*sin(theta/180*M_PI);
  double yStart = p.mooringRadiusToFairleadsFromCenter*cos(theta/180*M_PI);
  //End Position of Mooring Line
  double xEnd = p.mooringRadiusToAnchorFromCenter*sin(theta/180*M_PI);
  double yEnd = p.mooringRadiusToAnchorFromCenter*cos(theta/180*M_PI);

  //Coordinates of Fairlead in local frame of monopile
  mooringFairlead = monopile->getBody()->TransformPointLocalToParent(ChVector<>(xStart,yStart,p.distanceZPlatformCOGtoFairlead));

  qDebug() << "mooringFairlead x:" << mooringFairlead.x();
  qDebug() << "mooringFairlead y:" << mooringFairlead.y();
  qDebug() << "mooringFairlead z:" << mooringFairlead.z();

  //Coordinates of anchor in parent coordinates, y with negative sign because of rotation of monopile?
  mooringAnchor = ChVector<>(xEnd, yEnd, p.mooringPosAnchorZ);

  qDebug() << "mooringAnchor x:" << mooringAnchor.x();
  qDebug() << "mooringAnchor y:" << mooringAnchor.y();
  qDebug() << "mooringAnchor z:" << mooringAnchor.z();

  // Now, simply use BuildBeam to create a beam from a point to another:
  builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
    sectionCable,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
    p.mooringNrElements,      // the number of ChElementBeamANCF to create
    mooringFairlead,     // the 'A' point in space (beginning of beam)
    mooringAnchor        // the 'B' point in space (end of beam)
  );

  //create truss (fixed body)
  auto mtruss = std::make_shared<ChBody>();
  mtruss->SetBodyFixed(true);
  const ChVector<> pos = builder.GetLastBeamNodes().front()->GetPos();

  //constraint mooring to monopile
  constraintMooring = std::make_shared<ChLinkPointFrame>();
  constraintMooring->Initialize(builder.GetLastBeamNodes().front(), monopile->getBody());

  constraintMooring->SetAttachPositionInAbsoluteCoords(pos);
  //constraintMooring->SetAttachReferenceInAbsoluteCoords(builder.GetLastBeamNodes().GetCoord());

  system.Add(constraintMooring);

  //anchor constraint
  constraintAnchor = std::make_shared<ChLinkPointFrame>();
  constraintAnchor->Initialize(builder.GetLastBeamNodes().back(), mtruss);
  system.Add(constraintAnchor);

  //Apply Aquivalent weight force in water on mooring
  double buoyancyForcePerUnit;

  std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
  for(auto &element : beamElements){

      buoyancyForcePerUnit = mooringArea*p.rhoWater*p.g;

      // Add a distributed load along the cable element:
      auto mwrenchdis = std::make_shared<ChLoadBeamWrenchDistributed>(element);
      mwrenchdis->loader.SetForcePerUnit(ChVector<>(0, 0, buoyancyForcePerUnit));  // load per unit length
      loadContainer->Add(mwrenchdis);

  }

  qDebug() << "total buoyancy force of one mooring line" << buoyancyForcePerUnit * p.mooringUnstretchedLength;
  qDebug() << "total weight of one mooring line" << p.mooringUnstretchedLength*p.g*p.mooringDensityPerUnit;
}

void MooringLine::setRestLengthAndPosition(){

    double mooringLengthSetup = 0;

    std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
    //loop over elements to calculate the actual length of mooring line to determine necessary rest length for the pretension
    for(auto &element : beamElements){
        ChVector<> elementVector = element->GetNodeA()->GetPos()-element->GetNodeB()->GetPos();
        double currentLength = elementVector.Length();
        mooringLengthSetup = mooringLengthSetup + currentLength;
    }

    qDebug() << "determined setup length of mooring: " << mooringLengthSetup;

    double setupLengthOfElement = mooringLengthSetup/p.mooringNrElements;

    double frac = p.mooringUnstretchedLength/mooringLengthSetup;

    restLengthOfElement = setupLengthOfElement*frac;

    qDebug() << "setup length of cable element:" << setupLengthOfElement;
    qDebug() << "frac : " << frac;
    qDebug() << "mooring line length at rest:" << restLengthOfElement;

    double massTotal = 0;

    //Iterate over beam elements to set the rest length and rest position
    for(auto &element : beamElements){
        massTotal += element->GetMass();
        //qDebug() << "initial length"<<element->GetRestLength();

        element->SetRestLength(setupLengthOfElement*frac);
        CVector pos = CVecFromChVec(element->GetNodeA()->GetX0());
        CVector pos0 = CVecFromChVec(element->GetNodeB()->GetX0());

        CVector lvec = CVector(pos-pos0);
        lvec.Normalize();
        lvec *= setupLengthOfElement*frac;

        //element->GetNodeA()->SetD(-ChVecFromCVec(lvec));
        //element->GetNodeB()->SetD(ChVecFromCVec(lvec));

        pos = pos0+lvec;
        ChVector<> chvec(pos.x,pos.y,pos.z);
        element->GetNodeB()->SetX0(chvec);

        //qDebug() << "new rest length"<<CVector(pos-pos0).VAbs() <<frac;
    }

    qDebug() << "cable mass:" << massTotal;

}


void MooringLine::getTensionForce(){

    double strain, currentLength;
    std::shared_ptr<ChBeamSectionCable> sec;

    double totalLength = 0;

    qDebug() << "testing cable force";
    std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
    for (auto &element : beamElements){
        ChVector<> elementVector = element->GetNodeA()->GetPos()-element->GetNodeB()->GetPos();
        double currentLength = elementVector.Length();
        qDebug() << "length of element after init" << currentLength;
        strain = (currentLength-restLengthOfElement)/restLengthOfElement;
        sec =  element->GetSection();
        double tensionForce = strain*sec->E*sec->Area;
        qDebug() << "tensionForce: analytically" << strain*sec->E*sec->Area;
        totalLength = totalLength + currentLength;
    }

    qDebug() << "total length of mooring line after init: " << totalLength;
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
    glLineWidth(3);
    glBegin(GL_LINES);
        //Render Coordinate system of fairlead

        ChVector<> fairleadPos = builder.GetLastBeamNodes().front()->GetPos();
        ChVector<> fairleadD = builder.GetLastBeamNodes().front()->GetD();
        CVector fairleadPosCVec = CVecFromChVec(fairleadPos);
        //green: directional vector
        glColor4d(0,1,0,1);
        /*
        glVertex3d(fairleadPosCVec.x,fairleadPosCVec.y,fairleadPosCVec.z);
        CVector dVectorEnd = CVecFromChVec(fairleadPos+p.dVectorFactor*fairleadD);
        glVertex3d(dVectorEnd.x,dVectorEnd.y,dVectorEnd.z);
        qDebug() << "fairleadPos x:" << fairleadPosCVec.x;
        qDebug() << "fairleadPos y:" << fairleadPosCVec.y;
        qDebug() << "fairleadPos z:" << fairleadPosCVec.z;

        qDebug() << "dVectorEnd x:" << dVectorEnd.x;
        qDebug() << "dVectorEnd y:" << dVectorEnd.y;
        qDebug() << "dVectorEnd z:" << dVectorEnd.z;
        */
        for(auto &node : beamNodes){

            ChVector<> nodePos = node->GetPos();
            ChVector<> nodeD = node->GetD();
            CVector nodePosCVec = CVecFromChVec(nodePos);
            //green: directional vector
            glColor4d(0,1,0,1);
            glVertex3d(nodePosCVec.x,nodePosCVec.y,nodePosCVec.z);
            CVector dVectorEnd = CVecFromChVec(nodePos+p.dVectorFactor*nodeD);
            glVertex3d(dVectorEnd.x,dVectorEnd.y,dVectorEnd.z);
            /*
            qDebug() << "nodeD x:" << nodeD.x();
            qDebug() << "nodeD y:" << nodeD.y();
            qDebug() << "nodeD z:" << nodeD.z();
            */
            //qDebug() << "nodeD Length x:" << nodeD.Length();
        }

        //Render react forces inside the link of fairlead

        ChVector<> reactForceFairlead = constraintMooring->Get_react_force();
        //qDebug() << "reactForceFairlead: " << reactForceFairlead.Length();
        double tension = reactForceFairlead.Length()/mooringArea;
        //qDebug() << "mooringTension: " << tension;

        reactForceFairlead.Normalize();
        //red: react force
        glColor4d(1,0,0,1);
        glVertex3d(fairleadPosCVec.x,fairleadPosCVec.y,fairleadPosCVec.z);
        CVector reactionForceVectorEnd = CVecFromChVec(fairleadPos+p.dVectorFactor*reactForceFairlead);
        glVertex3d(reactionForceVectorEnd.x,reactionForceVectorEnd.y,reactionForceVectorEnd.z);

        //Render react forces inside the link of mooring
        ChVector<> reactForceMooring = constraintMooring->Get_react_force();
        reactForceMooring.Normalize();
        //blue: react force inside mooring
        glColor4d(0,0,1,1);
        glVertex3d(fairleadPosCVec.x,fairleadPosCVec.y,fairleadPosCVec.z);
        reactionForceVectorEnd = CVecFromChVec(fairleadPos-p.dVectorFactor*reactForceMooring);
        glVertex3d(reactionForceVectorEnd.x,reactionForceVectorEnd.y,reactionForceVectorEnd.z);

    glEnd();
}
