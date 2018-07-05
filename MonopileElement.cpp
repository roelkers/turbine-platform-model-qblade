
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "MonopileElement.h"


using namespace chrono;
using namespace chrono::fea;

std::shared_ptr<chrono::ChMarker> MonopileElement::getMarker() const
{
    return marker;
}

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
       true, //local_force
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

    ChVector<> markerVelocityAbs = marker->GetAbsFrame().GetPos_dt();
    ChVector<> markerVelocityDir = body->TransformDirectionParentToLocal(markerVelocityAbs);
    //double markerVelocityLength = markerVelocityAbs.Length();

    //qDebug() << "markerVelocityDir Length: "<< markerVelocityDir.Length();
    //qDebug() << "markerVelocity " << markerVelocityLength;

    double markerVelX = markerVelocityDir.x();
    double markerVelY = markerVelocityDir.y();
    double markerVelZ = markerVelocityDir.z();

    AinAbsoluteFrame = body->TransformPointLocalToParent(A);
    BinAbsoluteFrame = body->TransformPointLocalToParent(B);

//    ChVector<> vecABabs = BinAbsoluteFrame - AinAbsoluteFrame;
//    double projectedLengthXZ = sqrt(pow(vecABabs.x() ,2) + pow(vecABabs.z(),2));
//    double projectedLengthYZ = sqrt(pow(vecABabs.y() ,2) + pow(vecABabs.z(),2));
//    double projectedLengthXY = sqrt(pow(vecABabs.x() ,2) + pow(vecABabs.y(),2));

    //qDebug() << "projectedLengthXZ: " << projectedLengthXZ;
    //qDebug() << "projectedLengthYZ: " << projectedLengthYZ;

//    double areaXZ = projectedLengthXZ*2*p.towerRadius;
//    double areaYZ = projectedLengthYZ*2*p.towerRadius;
//    double areaXY = projectedLengthXY*2*p.towerRadius;

    double dragForceX;
    double dragForceY;
    //double dragForceZ;

    double buoyancyForceZ;
    //check if marker is submerged

    if(isSubmerged()){

        dragForceX = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelX*crossSectionArea;
        dragForceY = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelY*crossSectionArea;
        //dragForceZ = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelZ*areaXY;

        buoyancyForceZ = p.rhoWater*volume*p.g;
    }
    else{
        dragForceX = 0;
        dragForceY = 0;
        //dragForceZ = 0;

        buoyancyForceZ = 0;
    }

    ChVector<> dragForceVec = ChVector<>(dragForceX, dragForceY, 0);

    ChVector<> buoyancyForceVec = ChVector<>(0,0,buoyancyForceZ);

    //qDebug()<< "dragForceVec" << dragForceVec.Length();

    dampingForce->SetForce(dragForceVec,true);
    dampingForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);

    buoyancyForce->SetForce(buoyancyForceVec,false);
    buoyancyForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);
}

bool MonopileElement::isSubmerged(){
    return (marker->GetAbsCoord().pos.z() < p.seaLevel);
}

void MonopileElement::render(){

//    qDebug() << "render damping element";
    glLineWidth(3);

    CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
    //    qDebug()<< "markerPos.x :" << markerPos.x;
    //    qDebug()<< "markerPos.y :" << markerPos.y;
    //    qDebug()<< "markerPos.z :" << markerPos.z;

    ChVector<> dragForce = body->TransformDirectionLocalToParent(dampingForce->GetForce());
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
        CVector dragForceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+dragForce*p.forceLineFactor);
        glVertex3d(dragForceVecEnd.x,dragForceVecEnd.y,dragForceVecEnd.z);
        //light blue: buoyancyforce
//        glColor4d(0,0,0.5,1);
//        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
//        CVector buoyancyForceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+p.cSystemFactor*buoyancyForce->GetForce()*p.forceLineFactor);
//        glVertex3d(buoyancyForceVecEnd.x,buoyancyForceVecEnd.y,buoyancyForceVecEnd.z);
    glEnd();
}

