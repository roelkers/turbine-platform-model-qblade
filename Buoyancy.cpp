#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChMarker.h"
#include <chrono/core/ChVector.h>
#include <chrono/core/ChLog.h>
#include <chrono/core/ChCoordsys.h>
#include <chrono/assets/ChTexture.h>

#include <QDebug>
#include "../GlobalFunctions.h"

#include "Buoyancy.h"
#include "PlatformParams.h"

using namespace chrono;
using namespace chrono::fea;

Buoyancy::Buoyancy(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBody> monopile)

:p(p),
monopile(monopile),
loadContainer(loadContainer)
{
  //ChVector<> pos = monopile->GetPos();
  /*
  //ChVector<> towerPos = monopile->GetPos();
  ChFrameMoving<> frame = monopile->GetFrame_COG_to_abs();
  //Get rotation of frame as a quaternion
  ChQuaternion<> qmonopile = frame.GetRot();
  //Get unity vector in z direction
  ChVector<> zUnityVector = ChVector<>(0,0,1);
  //Rotate Coordinate system back
  ChQuaternion<> qcorrection = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);


  ChQuaternion<> qcombined = qmonopile* qcorrection;
  //Get vector in direction of tower axis by rotating vector around quaternion
  ChVector<> towerAxis = qcombined.Rotate(zUnityVector);
  */

  markerBottom = std::make_shared<ChMarker>();
  //Set Marker Position relative to local coordinate system
  ChCoordsys<> bottomCoordsys = ChCoordsys<>(monopile->TransformPointLocalToParent(ChVector<>(0,-0.5*p.towerHeight,0)));
  //Set marker parameters
  markerBottom->SetBody(monopile.get());
  markerBottom->Impose_Abs_Coord(bottomCoordsys);

  //markerBottom->SetPos(ChVector<>(0,0,-0.5*p.towerHeight));

  markerTop = std::make_shared<ChMarker>();
  //Set Marker Position relative to local coordinate system
  ChCoordsys<> topCoordsys = ChCoordsys<>(monopile->TransformPointLocalToParent(ChVector<>(0,0.5*p.towerHeight,0)));
  //Set marker parameters
  markerTop->SetBody(monopile.get());
  markerTop->Impose_Abs_Coord(topCoordsys);
  //ChVector<> vecE = markerBottom->GetAbsCoord().pos;
  //ChVector<> vecI = markerTop->GetAbsCoord().pos;

  qDebug() << "topMarker x: " << markerTop->GetAbsCoord().pos.x();
  qDebug() << "topMarker y: " << markerTop->GetAbsCoord().pos.y();
  qDebug() << "topMarker z: " << markerTop->GetAbsCoord().pos.z();

  qDebug() << "bottomMarker x: " << markerBottom->GetAbsCoord().pos.x();
  qDebug() << "bottomMarker y: " << markerBottom->GetAbsCoord().pos.y();
  qDebug() << "bottomMarker z: " << markerBottom->GetAbsCoord().pos.z();

  //computeMaximumBuoyancyForce();

  qDebug() << "maximum buoyancy force: " << maximumBuoyancyForce;

  //Init Buoyancy force with null vectors
  buoyancyForce = std::make_shared<ChLoadBodyForce> (
    monopile, //body
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

  qDebug() << "update markers";
  markerBottom->UpdateState();
  markerTop->UpdateState();

  ChVector<> seaLevelVector = ChVector<>(0,0,p.seaLevel);
  ChVector<> towerPos = monopile->GetPos();
  ChFrameMoving<> frame = monopile->GetFrame_COG_to_abs();
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
  //Check if the sea level plane is parallel to the tower axis,
  //this is the edge case. then the intersection point will be undefined.
  //Scalar Product:
  qDebug() << "quaternion data0:" << qmonopile.e0();
  qDebug() << "quaternion data1:" << qmonopile.e1();
  qDebug() << "quaternion data2:" << qmonopile.e2();
  qDebug() << "quaternion data3:" << qmonopile.e3();
  qDebug() << "towerAxis x: " << towerAxis.x();
  qDebug() << "towerAxis y: " << towerAxis.y();
  qDebug() << "towerAxis z: " << towerAxis.z();

  if((towerAxis^zUnityVector)==0){
    qDebug() << "Edge Case for Buoyancy" << "\n";
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
    vecE = markerBottom->GetAbsCoord().pos;
    vecI = markerTop->GetAbsCoord().pos;

    qDebug() << "r-Konstante:" << rConstant << "\n";
  }

  computeBuoyancy(vecE, vecI);

  qDebug() << "monopile x: " << monopile->GetPos().x();
  qDebug() << "monopile y: " << monopile->GetPos().y();
  qDebug() << "monopile z: " << monopile->GetPos().z();

  qDebug() << "topMarker x: " << markerTop->GetAbsCoord().pos.x();
  qDebug() << "topMarker y: " << markerTop->GetAbsCoord().pos.y();
  qDebug() << "topMarker z: " << markerTop->GetAbsCoord().pos.z();

  qDebug() << "bottomMarker x: " << markerBottom->GetAbsCoord().pos.x();
  qDebug() << "bottomMarker y: " << markerBottom->GetAbsCoord().pos.y();
  qDebug() << "bottomMarker z: " << markerBottom->GetAbsCoord().pos.z();
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

  ChVector<> towerPos = monopile->GetPos();

  //ChVector<> vecES = intersectionPoint- vecE;
  //ChVector<> vecIS = intersectionPoint- vecE;
  ChVector<> vecSE = vecE - intersectionPoint;
  ChVector<> vecSI = vecI - intersectionPoint;

  ChVector<> vecGE = vecE - towerPos;
  ChVector<> vecGS = intersectionPoint - towerPos;
  ChVector<> vecGI = vecI - towerPos;

  double force = 0;
  //check if distance to sea level is larger than distance to top of tower
  if(vecGS.Length() >= vecGI.Length() && vecGS.Length() >= vecGE.Length()){
    //sanity check if I and E are both below Sea level
    if(vecE.z() < p.seaLevel && vecI.z() < p.seaLevel){
    //tower completely submerged, buoyancy center is same as gravity center
    qDebug() << "completely submerged\n";
    buoyancyCenter = towerPos;
    //force = maximumBuoyancyForce;
    force = computeBuoyancyForce(p.towerHeight);
    qDebug() << "buoyancyForce completely submerged:" << force << "\n";
    }
    else{
    // in this case the tower is "flying", and we should not apply any buoyancy force
    qDebug() << "flying tower\n";
    force = 0;
    }
  }
  else{
    //Check which end of the tower is submerged, and which is above the sea
    if(vecE.z() > p.seaLevel){
      qDebug() << "partly submerged, E above sea level\n";
      //construct buoyancy volume from S to I
      buoyancyCenter = intersectionPoint + 0.5*vecSI;
      ChVector<> submergedVector = vecSI*2;
      qDebug() << "submergedVector" << submergedVector.Length();
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else if(vecI.z() > p.seaLevel){
      qDebug() << "partly submerged, I above sea level\n";
      //construct buoyancy volume from S to E
      buoyancyCenter = intersectionPoint + 0.5*vecSE;
      ChVector<> submergedVector = vecSE*2;
      qDebug() << "submergedVector" << submergedVector.Length();
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else qDebug() << "both E and I below sea level.This MUST not happen.\n";
  }

  qDebug() << "buoyancyForce:" << force << "\n";

  buoyancyForce->SetApplicationPoint(buoyancyCenter,false);

  buoyancyForce->SetForce(ChVector<>(0,0,force),false);
}
/*
double Buoyancy::computeMaximumBuoyancyForce(){
        //tower completely submerged, return volume of complete water cylinder
    double submergedVolumeMonopile =  M_PI*pow(p.towerRadius,2)*p.towerHeight;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}
*/
double Buoyancy::computeBuoyancyForce(double submergedLength){
    double submergedVolumeMonopile = M_PI*pow(p.towerRadius,2)*submergedLength;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}

