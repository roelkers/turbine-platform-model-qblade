#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/assets/ChTexture.h"

#include <QDebug>
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "Buoyancy.h"
#include "PlatformParams.h"
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

Buoyancy::Buoyancy(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile)

:p(p),
monopile(monopile),
loadContainer(loadContainer)
{

  //Init Buoyancy force with null vectors
  buoyancyForce = std::make_shared<ChLoadBodyForce> (
    monopile->getCylinder(), //body
    ChVector<>(0,0,0), //force in positive z direction
    false, //local_force
    ChVector<>(0,0,0), //local Gravity Center
    true //local point
  );

  //Update Buoyancy Force
  update();

  //Add load to container
  loadContainer->Add(buoyancyForce);
}

void Buoyancy::update(){

  monopile->updateMarkers();

  ChVector<> seaLevelVector = ChVector<>(0,0,p.seaLevel);
  ChVector<> towerPos = monopile->getCylinder()->GetPos();
  ChFrameMoving<> frame = monopile->getCylinder()->GetFrame_COG_to_abs();
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

  ChVector<> vecE;
  ChVector<> vecI;
  ChVector<> intersectionPoint;
  //Check if the sea level plane is parallel to the tower axis,
  //this is the edge case. then the intersection point will be undefined.
  //Scalar Product:
  /*
  qDebug() << "quaternion data0:" << qmonopile.e0();
  qDebug() << "quaternion data1:" << qmonopile.e1();
  qDebug() << "quaternion data2:" << qmonopile.e2();
  qDebug() << "quaternion data3:" << qmonopile.e3();
  qDebug() << "towerAxis x: " << towerAxis.x();
  qDebug() << "towerAxis y: " << towerAxis.y();
  qDebug() << "towerAxis z: " << towerAxis.z();
  */
  if((towerAxis^zUnityVector)==0){
    //qDebug() << "Edge Case for Buoyancy" << "\n";
    //intersection point is exactly on a line parallel to the z axis below the gravity
    //center. Point E and I are at the radius of the tower
    intersectionPoint = ChVector<>(towerPos.x(),towerPos.y(),p.seaLevel);

    vecE = ChVector<>(towerPos.x(),towerPos.y(),towerPos.z()-p.towerRadius);
    vecI = ChVector<>(towerPos.y(),towerPos.y(),towerPos.z()+p.towerRadius);
  }
  else{
    //Point E and I are at the top and bottom of the tower
    //Calculate Intersection point of sea level plane with algebraic equation
    const double rConstant = ((seaLevelVector-towerPos)^zUnityVector)/(towerAxis^zUnityVector);
    //Intersection point using straight line equation
    intersectionPoint = towerPos + towerAxis*rConstant;
    //Get Position of the Top and bottom via the body markers
    vecE = monopile->getMarkerBottom()->GetAbsCoord().pos;
    vecI = monopile->getMarkerTop()->GetAbsCoord().pos;
  }
  monopile->setIntersectionPoint(intersectionPoint);
  computeBuoyancy(vecE, vecI);
  /*
  qDebug() << "monopile x: " << towerPos.x();
  qDebug() << "monopile y: " << towerPos.y();
  qDebug() << "monopile z: " << towerPos.z();

  qDebug() << "topMarker x: " << monopile->getMarkerTop()->GetAbsCoord().pos.x();
  qDebug() << "topMarker y: " << monopile->getMarkerTop()->GetAbsCoord().pos.y();
  qDebug() << "topMarker z: " << monopile->getMarkerTop()->GetAbsCoord().pos.z();

  qDebug() << "bottomMarker x: " << monopile->getMarkerBottom()->GetAbsCoord().pos.x();
  qDebug() << "bottomMarker y: " << monopile->getMarkerBottom()->GetAbsCoord().pos.y();
  qDebug() << "bottomMarker z: " << monopile->getMarkerBottom()->GetAbsCoord().pos.z();
  */
}

void Buoyancy::computeBuoyancy(ChVector<> vecE, ChVector<> vecI){
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
  ChVector<> intersectionPoint = monopile->getIntersectionPoint();
  ChVector<> towerPos = monopile->getCylinder()->GetPos();

  //ChVector<> vecES = intersectionPoint- vecE;
  //ChVector<> vecIS = intersectionPoint- vecE;
  ChVector<> vecSE = vecE - intersectionPoint;
  ChVector<> vecSI = vecI - intersectionPoint;

  ChVector<> vecGE = vecE - towerPos;
  ChVector<> vecGS = intersectionPoint - towerPos;
  ChVector<> vecGI = vecI - towerPos;
  ChVector<> vecEI = vecI - vecE;

  double force = 0;
  ChVector<> buoyancyCenter;
  //check if distance to sea level is larger than distance to top of tower
  if(vecGS.Length() >= vecGI.Length() && vecGS.Length() >= vecGE.Length()){
    //sanity check if I and E are both below Sea level
    if(vecE.z() < p.seaLevel && vecI.z() < p.seaLevel){
    //tower completely submerged, buoyancy center is same as gravity center
    monopile->submergedPart = Monopile::BOTH;
    //qDebug() << "completely submerged\n";
    buoyancyCenter = towerPos;
    monopile->setSubmergedVector(vecEI);
    force = computeBuoyancyForce(p.towerHeight);
    //qDebug() << "buoyancyForce completely submerged:" << force << "\n";
    }
    else{
    // in this case the tower is "flying", and we should not apply any buoyancy force
    monopile->submergedPart= Monopile::NONE;
    monopile->setSubmergedVector(ChVector<>(0,0,0));
    force = 0;
    }
  }
  else{
    //Check which end of the tower is submerged, and which is above the sea
    if(vecE.z() > p.seaLevel){
      //qDebug() << "partly submerged, E above sea level\n";
      monopile->submergedPart = Monopile::NACELLE;
      //construct buoyancy volume from S to I
      buoyancyCenter = intersectionPoint + 0.5*vecSI;
      //ChVector<> submergedVector = vecSI*2;
      ChVector<> submergedVector = vecSI;
      monopile->setSubmergedVector(submergedVector);
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else if(vecI.z() > p.seaLevel){
      //qDebug() << "partly submerged, I above sea level\n";
      //construct buoyancy volume from S to E
      monopile->submergedPart = Monopile::BALLAST;
      buoyancyCenter = intersectionPoint + 0.5*vecSE;
      //ChVector<> submergedVector = vecSE*2;
      ChVector<> submergedVector = vecSE;
      monopile->setSubmergedVector(submergedVector);
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else qDebug() << "both E and I below sea level.This MUST not happen.\n";
  }

  //qDebug() << "submerged Part of Monopile:" << monopile->submergedPart;
  //qDebug() << "buoyancyForce:" << force << "\n";

  monopile->setBuoyancyCenter(buoyancyCenter);

  buoyancyForce->SetApplicationPoint(monopile->getBuoyancyCenter(),false);

  buoyancyForce->SetForce(ChVector<>(0,0,force),false);


}

double Buoyancy::computeBuoyancyForce(double submergedLength){
    double submergedVolumeMonopile = M_PI*pow(p.towerRadius,2)*submergedLength;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}

void Buoyancy::render(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //red/green: intersection Point
        glColor4d(1,1,0,1);
        CVector isPos = CVecFromChVec(monopile->getIntersectionPoint());
        glVertex3d(isPos.x,isPos.y,isPos.z);
        //blue/green: buoyancy center
        glColor4d(0,1,1,1);
        CVector buoyancyCenterPos = CVecFromChVec(monopile->getBuoyancyCenter());
        glVertex3d(buoyancyCenterPos.x,buoyancyCenterPos.y,buoyancyCenterPos.z);
    glEnd();
}
