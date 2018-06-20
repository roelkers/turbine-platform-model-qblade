
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "MonopileElement.h"


using namespace chrono;
using namespace chrono::fea;

MonopileElement::MonopileElement(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<chrono::ChBody> body, double length, ChVector<> A, ChVector<> B, double crossSectionArea, double volume)
    :p(p),
     body(body),
     A(body->TransformPointParentToLocal(A)),
     B(body->TransformPointParentToLocal(B)),
     AinAbsoluteFrame(A),
     BinAbsoluteFrame(B),
     length(length),
     crossSectionArea(crossSectionArea),
     volume(volume)
{
    //Set marker in center of element
    ChVector<> markerPosition = (A + B)/2;

    marker = std::make_shared<ChMarker>();
    ChCoordsys<> markerCoordsys = ChCoordsys<>(ChVector<>(markerPosition),body->GetRot());
    marker->SetBody(body.get());
    marker->Impose_Abs_Coord(markerCoordsys);

    //ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
    //marker->SetRot(qSetup);

    dampingForce = std::make_shared<ChLoadBodyForce> (
       body, //body
       ChVector<>(0,0,0), //initialize
       false, //not a local_force
       marker->GetPos(), //apply force at marker position
       false //not a local point
    );

    buoyancyForce = std::make_shared<ChLoadBodyForce> (
       body, //body
       ChVector<>(0,0,0), //initialize
       false, //not a local_force
       marker->GetPos(), //apply force at marker position
       false //not a local point
    );

    loadContainer->Add(dampingForce);
    loadContainer->Add(buoyancyForce);

}

void MonopileElement::update(){

    marker->UpdateState();

    double markerVelX = marker->GetAbsCoord_dt().pos.x();
    double markerVelY = marker->GetAbsCoord_dt().pos.y();
    double markerVelZ = marker->GetAbsCoord_dt().pos.z();

    AinAbsoluteFrame = body->TransformPointLocalToParent(A);
    BinAbsoluteFrame = body->TransformPointLocalToParent(B);

    ChVector<> vecABabs = BinAbsoluteFrame - AinAbsoluteFrame;
    double projectedLengthXZ = sqrt(pow(vecABabs.x() ,2) + pow(vecABabs.z(),2));
    double projectedLengthYZ = sqrt(pow(vecABabs.y() ,2) + pow(vecABabs.z(),2));
    double projectedLengthXY = sqrt(pow(vecABabs.x() ,2) + pow(vecABabs.y(),2));

    //qDebug() << "projectedLengthXZ: " << projectedLengthXZ;
    //qDebug() << "projectedLengthYZ: " << projectedLengthYZ;

    double areaXZ = projectedLengthXZ*2*p.towerRadius;
    double areaYZ = projectedLengthYZ*2*p.towerRadius;
    double areaXY = projectedLengthXY*2*p.towerRadius;

    double dragForceX;
    double dragForceY;
    double dragForceZ;

    double buoyancyForceZ;
    //check if marker is submerged
    if(marker->GetAbsCoord().pos.z() < p.seaLevel){
//        forceX = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelX*crossSectionArea;
//        forceY = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelY*crossSectionArea;
//        forceZ = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelZ*crossSectionArea;

        dragForceX = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelX*areaYZ;
        dragForceY = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelY*areaXZ;
        dragForceZ = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelZ*areaXY;

        buoyancyForceZ = p.rhoWater*volume*p.g;
//        qDebug() << "element submerged";
//        qDebug() << "-0.5*p.rhoWater*p.dragCoefficientCylinderLateral" << -0.5*p.rhoWater*p.dragCoefficientCylinderLateral;
//        qDebug() << "markerVelY*crossSectionArea" << markerVelY*crossSectionArea;
    }
    else{
        dragForceX = 0;
        dragForceY = 0;
        dragForceZ = 0;

        buoyancyForceZ = 0;
    }

    //ChVector<> force = ChVector<>(forceX,forceY,0);
    ChVector<> dragForceVec = ChVector<>(dragForceX, dragForceY, dragForceZ);
//    qDebug() << "calc: damping force x: " << forceX;
//    qDebug() << "calc: damping force y: " << forceY;
//    qDebug() << "calc: damping force z: " << forceZ;
//    qDebug() << "damping force elem: " << force.Length();
    ChVector<> buoyancyForceVec = ChVector<>(0,0,buoyancyForceZ);

    dampingForce->SetForce(dragForceVec,false);
    dampingForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);

    buoyancyForce->SetForce(buoyancyForceVec,false);
    buoyancyForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);
}

void MonopileElement::render(){

//    qDebug() << "render damping element";
    glLineWidth(3);

    CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
    //    qDebug()<< "markerPos.x :" << markerPos.x;
    //    qDebug()<< "markerPos.y :" << markerPos.y;
    //    qDebug()<< "markerPos.z :" << markerPos.z;

    ChVector<> force = dampingForce->GetForce();
    //ChVector<> force = body->TransformDirectionLocalToParent(dampingForce->GetForce());

//    qDebug() << " damping force x: " << force.x();
//    qDebug() << " damping force y: " << force.y();
//    qDebug() << " damping force z: " << force.z();
    glPointSize(10);
    glBegin(GL_POINTS);
        //red/light green/blue: marker
        glColor4d(1,0.5,1,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
        //Draw Points A and B
        //red: point A
        CVector Apos = CVecFromChVec(AinAbsoluteFrame);
        glColor4d(1,0,0,1);
        glVertex3d(Apos.x,Apos.y,Apos.z);
        //green: point B
        CVector Bpos = CVecFromChVec(BinAbsoluteFrame);
        glColor4d(0,1,0,1);
        glVertex3d(Bpos.x,Bpos.y,Bpos.z);
    glEnd();
    glBegin(GL_LINES);
        //Draw element: black
        glColor4d(0,0,0,1);
        glVertex3d(Apos.x,Apos.y,Apos.z);
        glVertex3d(Bpos.x,Bpos.y,Bpos.z);
//        //Draw Coordinate system of marker
//        ChCoordsys<> markerCoord = marker->GetAbsCoord();
//        //red: x-axis
//        glColor4d(1,0,0,1);
//        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
//        CVector xAxisEnd = CVecFromChVec(markerCoord.pos+p.cSystemFactor*markerCoord.rot.GetXaxis());
//        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
//        //green: y-axis
//        glColor4d(0,1,0,1);
//        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
//        CVector yAxisEnd = CVecFromChVec(markerCoord.pos+p.cSystemFactor*markerCoord.rot.GetYaxis());
//        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
//        //blue: z-axis
//        glColor4d(0,0,1,1);
//        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
//        CVector zAxisEnd = CVecFromChVec(markerCoord.pos+p.cSystemFactor*markerCoord.rot.GetZaxis());
//        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);
        //light red/light green/ blue: damping force
        glColor4d(1,0.5,0.5,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
        CVector forceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+p.cSystemFactor*force/1000);
        glVertex3d(forceVecEnd.x,forceVecEnd.y,forceVecEnd.z);
    glEnd();
}

