
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLinkMate.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "PlatformParams.h"
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

chrono::ChVector<> Monopile::getBuoyancyCenter() const
{
    return buoyancyCenter;
}

chrono::ChVector<> Monopile::getIntersectionPoint() const
{
    return intersectionPoint;
}

chrono::ChVector<> Monopile::getSubmergedVector() const
{
    return submergedVector;
}

void Monopile::setSubmergedVector(const chrono::ChVector<> &value)
{
    submergedVector = value;
}

Monopile::Monopile(ChSystem &system, PlatformParams p)
    :p(p)
{

    qDebug() << "creating cylinder";
    cylinder = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
    //set cylinder to setup position
    cylinder->SetPos(p.towerSetupPos);

    //qDebug() << "rotate cylinder";
    //Setup location of cylinder
    ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
    cylinder->SetRot(qSetup);
    system.Add(cylinder);

    qDebug() << "cylinder mass: " << cylinder->GetMass();

//    //ChVector<> pos = cylinder->GetPos();

//    //ChVector<> towerPos = cylinder->GetPos();
//    ChFrameMoving<> frame = cylinder->GetFrame_COG_to_abs();
//    //Get rotation of frame as a quaternion
//    ChQuaternion<> qcylinder = frame.GetRot();
//    //Get unity vector in z direction
//    ChVector<> zUnityVector = ChVector<>(0,0,1);
//    //Rotate Coordinate system back
//    ChQuaternion<> qcorrection = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);

//    ChQuaternion<> qcombined = qcylinder* qcorrection;
//    //Get vector in direction of tower axis by rotating vector around quaternion
//    ChVector<> towerAxis = qcombined.Rotate(zUnityVector);

    markerGravityCenter = std::make_shared<ChMarker>();
    //Set Marker Position relative to local coordinate system
    ChCoordsys<> gravCenterCoordsys = ChCoordsys<>(ChVector<>(0,0,-0.5*p.towerHeight)+calculateGravityCenterFromBottom());
    //Set marker parameters
    markerGravityCenter->SetBody(cylinder.get());
    markerGravityCenter->Impose_Abs_Coord(gravCenterCoordsys);
}

void Monopile::addNacelleAndBallast(ChSystem &system){

    //Create ballast on the bottom of the cylinder
    ballast = std::make_shared<ChBody>();
    ballast->SetMass(p.ballastMass);
    system.Add(ballast);
    //Move to position in local frame, on the bottom end
    ChVector<> ballastPos = cylinder->TransformPointLocalToParent(ChVector<>(0,-0.5*p.towerHeight,0)); //local frame to transform
    ballast->SetPos(ballastPos);
    //ballast->SetRot(qSetup);
    //qDebug() << "ballast mass: " << ballast->GetMass();
    //ballast constraint, attach to cylinder
    std::shared_ptr<ChLinkMateFix> constraint_ballast = std::make_shared<ChLinkMateFix>();
    constraint_ballast->Initialize(ballast, cylinder);
    system.Add(constraint_ballast);

    //Create nacelle mass on the top of the monopile

    nacelle = std::make_shared<ChBody>();
    nacelle->SetMass(p.nacelleMass);
    system.Add(nacelle);
    //Move to position in local frame, on the top end
    ChVector<> nacellePos = cylinder->TransformPointLocalToParent(ChVector<>(0,0.5*p.towerHeight,0)); //local frame to transform
    nacelle->SetPos(nacellePos);
    //nacelle->SetRot(qSetup);
    //qDebug() << "nacelle mass: " << nacelle->GetMass();
    //nacelle constraint, attach to monopile
    std::shared_ptr<ChLinkMateFix> constraint_nacelle = std::make_shared<ChLinkMateFix>();
    constraint_nacelle->Initialize(nacelle, cylinder);
    system.Add(constraint_nacelle);
}

void Monopile::update(){
    // Skizze:
    // G: Center of gravity
    // S: intersection point at water surface
    // B: buoyancy Center
    // E: bottom
    // I: top
    // -----------I------------
    //            |
    //            |
    //            |
    //            |
    //            G
    //            |
    //~~~~~~~~~~~~S~~~~~~~~~~~~
    //            |
    //            B
    //            |
    //------------E-------------

    markerGravityCenter->UpdateState();

    ChVector<> seaLevelVector = ChVector<>(0,0,p.seaLevel);
    ChVector<> towerPos = cylinder->GetPos();
    ChFrameMoving<> frame = cylinder->GetFrame_COG_to_abs();
    //Get rotation of frame as a quaternion
    ChQuaternion<> qmonopile = frame.GetRot();
    //Rotate Coordinate system back
    ChQuaternion<> qcorrection = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);

    ChQuaternion<> qcombined = qmonopile* qcorrection;
    //qDebug() << "qcombined:" << qcombined << "\n";
    //Get unity vector in z direction
    ChVector<> zUnityVector = ChVector<>(0,0,1);
    //Get vector in direction of tower axis by rotating vector around quaternion
    ChVector<> towerAxis = qcombined.Rotate(zUnityVector);

    //Point E and I are at the top and bottom of the tower
    //Calculate Intersection point of sea level plane with algebraic equation
    const double rConstant = ((seaLevelVector-towerPos)^zUnityVector)/(towerAxis^zUnityVector);
    //Intersection point using straight line equation
    intersectionPoint = towerPos + towerAxis*rConstant;
    //Get Position of the Top and bottom via the body markers
    ChVector<> vecE = ballast->GetPos();
    ChVector<> vecI = nacelle->GetPos();

    ChVector<> vecSE = vecE - intersectionPoint;
    ChVector<> vecSI = vecI - intersectionPoint;

    ChVector<> vecGE = vecE - towerPos;
    ChVector<> vecGS = intersectionPoint - towerPos;
    ChVector<> vecGI = vecI - towerPos;
    ChVector<> vecEI = vecI - vecE;

    //check if distance to sea level is larger than distance to top of tower
    if(vecGS.Length() >= vecGI.Length() && vecGS.Length() >= vecGE.Length()){
      //sanity check if I and E are both below Sea level
      if(vecE.z() < p.seaLevel && vecI.z() < p.seaLevel){
          //tower completely submerged, buoyancy center is same as gravity center
          submergedPart = Monopile::BOTH;
          //qDebug() << "completely submerged\n";
          buoyancyCenter = towerPos;
          submergedVector = vecEI;
          }
          else{
          // in this case the tower is "flying", and we should not apply any buoyancy force
          submergedPart= Monopile::NONE;
          submergedVector = ChVector<>(0,0,0);
      }
    }
    else{
      //Check which end of the tower is submerged, and which is above the sea
      if(vecE.z() > p.seaLevel){
        //qDebug() << "partly submerged, E above sea level\n";
        submergedPart = Monopile::NACELLE;
        //construct buoyancy volume from S to I
        buoyancyCenter = intersectionPoint + 0.5*vecSI;
        submergedVector = vecSI;
      }
      else if(vecI.z() > p.seaLevel){
        //qDebug() << "partly submerged, I above sea level\n";
        //construct buoyancy volume from S to E
        submergedPart = Monopile::BALLAST;
        buoyancyCenter = intersectionPoint + 0.5*vecSE;
        submergedVector = vecSE;
      }
      else qDebug() << "both E and I below sea level.This MUST not happen.\n";
    }
}

void Monopile::render(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //blue: monopile
        glColor4d(0,0,1,1);
        CVector monopilePos = CVecFromChVec(cylinder->GetPos());
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        //red/blue: ballast Body
        glColor4d(1,0,1,1);
        CVector ballastPos = CVecFromChVec(ballast->GetPos());
        glVertex3d(ballastPos.x, ballastPos.y, ballastPos.z);
        //light red/green: nacelle Body
        glColor4d(0.5,1,0,1);
        CVector nacellePos = CVecFromChVec(nacelle->GetPos());
        glVertex3d(nacellePos.x, nacellePos.y, nacellePos.z); 
        //light red/blue: gravity center
        glColor4d(0,0.5,1,1);
        CVector gravityCenterPos = CVecFromChVec(markerGravityCenter->GetAbsCoord().pos);
        glVertex3d(gravityCenterPos.x,gravityCenterPos.y,gravityCenterPos.z);
    glEnd();

    glPointSize(0.1);
    glBegin(GL_LINES);
        //Draw axis of tower
        glColor4d(0,0,0,1);
        glVertex3d(nacellePos.x,nacellePos.y,nacellePos.z);
        glVertex3d(ballastPos.x,ballastPos.y,ballastPos.z);
        //Draw Coordinate system of monopile
        ChCoordsys<> monopileCoord = cylinder->GetCoord();
        //red: x-axis
        glColor4d(1,0,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector xAxisEnd = CVecFromChVec(cylinder->GetPos()+p.cSystemFactor*monopileCoord.rot.GetXaxis());
        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
        //green: y-axis
        glColor4d(0,1,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector yAxisEnd = CVecFromChVec(cylinder->GetPos()+p.cSystemFactor*monopileCoord.rot.GetYaxis());
        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
        //blue: z-axis
        glColor4d(0,0,1,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector zAxisEnd = CVecFromChVec(cylinder->GetPos()+p.cSystemFactor*monopileCoord.rot.GetZaxis());
        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);
    glEnd();
}

ChVector<> Monopile::calculateGravityCenterFromBottom(){
    double massTotal = cylinder->GetMass()+p.nacelleMass+p.ballastMass;
    double areaMonopile = pow(p.towerRadius,2)*M_PI;

    double xs = (0.5*p.towerDensity*areaMonopile*pow(p.towerHeight,2)+p.towerHeight*p.nacelleMass)/massTotal;
    qDebug() << "xs : " << xs;

    return ChVector<>(0,0,xs);

}

