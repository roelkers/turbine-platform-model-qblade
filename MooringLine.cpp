#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "chrono/physics/ChLinkMate.h"

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
  double mooringLengthSetup = sqrt(pow(p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords,2) + pow(p.mooringAnchorRadiusFromFairlead,2));

  std::shared_ptr<ChBeamSectionCable> sectionCable = std::make_shared<ChBeamSectionCable>();
  sectionCable->SetDiameter(p.mooringDiameter);

  mooringArea = M_PI*pow(p.mooringDiameter,2)/4;

  sectionCable->SetArea(mooringArea);

  eModMooring = p.mooringStiffness*mooringLengthSetup/mooringArea;

  sectionCable->SetYoungModulus(eModMooring);
  sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);

  //Starting Position of Mooring Line on the Monopile
  double xStart = p.towerRadius*sin(theta/180*M_PI);
  double yStart = p.towerRadius*cos(theta/180*M_PI);
  //End Position of Mooring Line
  double xEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*sin(theta/180*M_PI);
  double yEnd = (p.towerRadius+p.mooringAnchorRadiusFromFairlead)*cos(theta/180*M_PI);

  //double length_test = sqrt(pow((xEnd-xStart),2) + pow((yEnd-yStart),2) + pow((p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords),2));
  //double distance_test = sqrt(pow(xStart,2)+pow(yStart,2));

  qDebug() << "mooringLengthSetup :" << mooringLengthSetup;
  //qDebug() << "length mooring check:" << length_test;
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

  //TRY here: mooringFairlead vector instead
  const ChVector<> pos = builder.GetLastBeamNodes().front()->GetPos();

  //Create fairlead body
  fairleadBody = std::make_shared<ChBody>();
  fairleadBody->SetMass(p.fairleadMass);
  system.Add(fairleadBody);
  fairleadBody->SetPos(pos);
  qDebug() << "fairlead mass: " << fairleadBody->GetMass();

  //constraint fairlead to monopile
  constraintFairlead = std::make_shared<ChLinkMateFix>();
  constraintFairlead->Initialize(fairleadBody, monopile->getCylinder());
  system.Add(constraintFairlead);

  //constraint mooring to fairlead
  constraintMooring = std::make_shared<ChLinkPointFrame>();
  constraintMooring->Initialize(builder.GetLastBeamNodes().front(), fairleadBody);

  constraintMooring->SetAttachPositionInAbsoluteCoords(pos);

  //constraint_fairlead->SetAttachPositionInBodyCoords(mooringFairlead);
  //ChCoordsys<> mooringFairleadCoord = ChCoordsys<>(mooringFairlead);
  //constraint_fairlead->SetAttachReferenceInAbsoluteCoords(mooringFairleadCoord);
  system.Add(constraintMooring);

  //anchor constraint
  constraintAnchor = std::make_shared<ChLinkPointFrame>();
  constraintAnchor->Initialize(builder.GetLastBeamNodes().back(), mtruss);
  system.Add(constraintAnchor);

}
/*
void MooringLine::setRestLengthAndPosition(){

    double mooringLengthSetup = sqrt(pow(p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords,2) + pow(p.mooringAnchorRadiusFromFairlead,2));
    qDebug()<< "mooringLengthSetup: " << mooringLengthSetup ;

    //Calculate actual length after initialization, this is necessary because the mooring lines
    //are not in the setup position (at the coordinate origin) anymore, so we need to account for that
    ChVector<> mooringActual = mooringAnchor - mooringFairlead;

    double mooringLengthInit = mooringActual.Length();
    qDebug()<< "mooringLengthInit: " << mooringLengthInit ;

    //thats the delta l for the setup position
    double deltal = p.mooringPreTensionForce/p.mooringStiffness;
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
*/
/*
double MooringLine::GetTensionForceAt(double pos){
    if (pos > 1) pos = 1.0;
    if (pos < 0) pos = 0.0;

    double strain, currentLength;
    std::shared_ptr<ChBeamSectionCable> sec;

    for (int i=0;i<Elements.size();i++){
        if (Elements.at(i)->normalizedLengthA <= pos && pos <= Elements.at(i)->normalizedLengthB){
            currentLength = CVector(Elements.at(i)->m_Nodes[0]->Origin-Elements.at(i)->m_Nodes[1]->Origin).VAbs();
            strain = (currentLength-initialLength)/initialLength;
            sec =  Elements.at(i)->GetSection();
        }
    }
    return strain*sec->E*sec->Area;
}
*/

void MooringLine::setRestLengthAndPosition(){

    double mooringLengthSetup = sqrt(pow(p.mooringPosBottomZ-p.mooringPosFairleadZInBodyCoords,2) + pow(p.mooringAnchorRadiusFromFairlead,2));
    qDebug()<< "mooringLengthSetup: " << mooringLengthSetup ;

    double setupLengthOfElement = mooringLengthSetup/p.mooringNrElements;

    double deltal = p.mooringPreTensionForce/p.mooringStiffness;
    double frac = (mooringLengthSetup-deltal)/mooringLengthSetup;

    qDebug() << "setup length:" << setupLengthOfElement;
    qDebug() << "frac : " << frac;
    qDebug() << "res:" << setupLengthOfElement*frac;

    //Iterate over beam elements to set the rest length and rest position
    std::vector<std::shared_ptr<ChElementCableANCF>> beamElements = builder.GetLastBeamElements();
    for(auto &element : beamElements){

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
        //glVertex3d(nodePos.x,nodePos.y,nodePos.z);
    }
    glEnd();
    glLineWidth(1.5);
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
            qDebug() << "nodeD Length x:" << nodeD.Length();
        }

        //Render react forces inside the link of fairlead

        ChVector<> reactForceFairlead = constraintFairlead->Get_react_force();
        qDebug() << "reactForceFairlead: " << reactForceFairlead.Length();
        double tension = reactForceFairlead.Length()/mooringArea;
        qDebug() << "mooringTension: " << tension;

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
