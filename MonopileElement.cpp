
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

    dragForce = std::make_shared<ChLoadBodyForce> (
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

    addedDampingForce = std::make_shared<ChLoadBodyForce> (
        body, //body
        ChVector<>(0,0,0), //initialize
        true, //local_force
        marker->GetPos(), //apply force at marker position
        false //not a local point
    );

    loadContainer->Add(dragForce);
    loadContainer->Add(buoyancyForce);

}

double MonopileElement::update(double seaLevel, double time){

    marker->UpdateState();

    ChVector<> markerVelocityAbs = marker->GetAbsFrame().GetPos_dt();
    ChVector<> markerVelocityDir = body->TransformDirectionParentToLocal(markerVelocityAbs);

    double markerVelX = markerVelocityDir.x();
    double markerVelY = markerVelocityDir.y();
    double markerVelZ = markerVelocityDir.z();

    AinAbsoluteFrame = body->TransformPointLocalToParent(A);
    BinAbsoluteFrame = body->TransformPointLocalToParent(B);

    double forceX;
    double forceY;
    double forceZ;

    double dragForceX;
    double dragForceY;
    double dragForceZ;

    double addedDampingForceX;
    double addedDampingForceY;
    double addedDampingForceZ;

    double buoyancyForceZ;
    //check if marker is submerged

    double zElement = marker->GetAbsFrame().GetPos().z();
    double omega = 2*PI/p.wavePeriod;

    waveVelocity = p.waveAmplitude*omega*exp(2*PI/p.waveLength*(zElement-seaLevel))*cos(-omega*time);
    ChVector<> waveVelocityAbsVec = ChVector<>(waveVelocity,0,0);
    waveVelocityLocalVec = body->TransformDirectionParentToLocal(waveVelocityAbsVec);
    qDebug() << "wave Velocity" << waveVelocity;

     if(isSubmerged(seaLevel)){

        double signX = 0;
        double signY = 0;

        if(markerVelX - waveVelocityLocalVec.x() >= 0) signX = 1;
        if(markerVelX - waveVelocityLocalVec.x() < 0 ) signX = -1;
        if(markerVelY - waveVelocityLocalVec.y()  >= 0) signY = 1;
        if(markerVelY - waveVelocityLocalVec.y()  < 0 ) signY = -1;
//        if(waveVelocity >= 0) signX = 1;
//        if(waveVelocity < 0 ) signX = -1;
        if(markerVelY >= 0) signY = 1;
        if(markerVelY < 0 ) signY = -1;

//        qDebug() << " signX" << signX;
//        qDebug() << " waveVelocity - markerVelX" << waveVelocity - markerVelX;

        //wave is travelling in positive x direction
        dragForceX = -signX* 0.5*p.rhoWater*p.dragCoefficientCylinderLateral*pow(markerVelX - waveVelocityLocalVec.x(),2)*crossSectionArea;
        dragForceY = -signY* 0.5*p.rhoWater*p.dragCoefficientCylinderLateral*pow(markerVelY  -waveVelocityLocalVec.y(),2)*crossSectionArea;
        dragForceZ = 0;

        buoyancyForceZ = p.rhoWater*volume*p.g;
    }
    else{
        dragForceX = 0;
        dragForceY = 0;

        buoyancyForceZ = 0;
    }

    //ChVector<> dragForceVec = ChVector<>(forceX, forceY, forceZ);
    ChVector<> dragForceVec = ChVector<>(dragForceX, dragForceY, dragForceZ);

    ChVector<> buoyancyForceVec = ChVector<>(0,0,buoyancyForceZ);

    qDebug() << "dragForceX" << dragForceX;
//    qDebug()<< "dragForceVec" << dragForceVec.Length();
//    qDebug() << "waveVelocity" << waveVelocity;

    dragForce->SetForce(dragForceVec,true);
    dragForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);

    buoyancyForce->SetForce(buoyancyForceVec,false);
    buoyancyForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);

    return dragForceX;
}

bool MonopileElement::isSubmerged(double seaLevel){
    return (marker->GetAbsCoord().pos.z() < seaLevel);
}

void MonopileElement::render(){

//    qDebug() << "render damping element";
    glLineWidth(3);

    CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
    //    qDebug()<< "markerPos.x :" << markerPos.x;
    //    qDebug()<< "markerPos.y :" << markerPos.y;
    //    qDebug()<< "markerPos.z :" << markerPos.z;

    ChVector<> dragForceAbs = body->TransformDirectionLocalToParent(dragForce->GetForce());
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

        //light red/light green/ blue: drag force
        glColor4d(1,0.5,0.5,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
        CVector dragForceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+dragForceAbs);
        glVertex3d(dragForceVecEnd.x,dragForceVecEnd.y,dragForceVecEnd.z);


        glColor4d(0,0.7,1,0.5);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
        CVector waveVelVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+ChVector<>(waveVelocity*1000,0,0));
        glVertex3d(waveVelVecEnd.x,waveVelVecEnd.y,waveVelVecEnd.z);

//        qDebug() << "addedDampingForceAbs.x()" << addedDampingForceAbs.x();
//        qDebug() << "addedDampingForceAbs.y()" << addedDampingForceAbs.y();
//        qDebug() << "addedDampingForceAbs.z()" << addedDampingForceAbs.z();

        //light blue: buoyancyforce
//        glColor4d(0,0,0.5,1);
//        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
//        CVector buoyancyForceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+p.cSystemFactor*buoyancyForce->GetForce()*p.forceLineFactor);
//        glVertex3d(buoyancyForceVecEnd.x,buoyancyForceVecEnd.y,buoyancyForceVecEnd.z);
    glEnd();
}

