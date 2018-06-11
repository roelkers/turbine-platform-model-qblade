
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "DampingElement.h"


using namespace chrono;
using namespace chrono::fea;

DampingElement::DampingElement(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile, double length, ChVector<> A, ChVector<> B, double crossSectionArea)
    :p(p),
     monopile(monopile),
     A(A),
     B(B),
     length(length),
     crossSectionArea(crossSectionArea)
{
    //Set marker in center of element
    ChVector<> markerPosition = (A + B)/2;

    marker = std::make_shared<ChMarker>();
    ChCoordsys<> markerCoordsys = ChCoordsys<>(ChVector<>(markerPosition),monopile->getCylinder()->GetRot());
    marker->SetBody(monopile->getCylinder().get());
    marker->Impose_Abs_Coord(markerCoordsys);

    //ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
    //marker->SetRot(qSetup);

    dampingForce = std::make_shared<ChLoadBodyForce> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       false, //not a local_force
       marker->GetPos(), //apply force at marker position
       false //not a local point
    );

    loadContainer->Add(dampingForce);

}

void DampingElement::update(){

    marker->UpdateState();

    double markerVelX = marker->GetAbsCoord_dt().pos.x();
    double markerVelY = marker->GetAbsCoord_dt().pos.y();
    double markerVelZ = marker->GetAbsCoord_dt().pos.z();

    //double markerVelX = marker->GetPos_dt().x();
    //double markerVelY = marker->GetPos_dt().y();

    double forceX;
    double forceY;
    double forceZ;
    //check if marker is submerged
    if(marker->GetAbsCoord().pos.z() < p.seaLevel){
        forceX = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelX*crossSectionArea;
        forceY = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelY*crossSectionArea;
        forceZ = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelZ*crossSectionArea;

//        qDebug() << "element submerged";
//        qDebug() << "-0.5*p.rhoWater*p.dragCoefficientCylinderLateral" << -0.5*p.rhoWater*p.dragCoefficientCylinderLateral;
//        qDebug() << "markerVelY*crossSectionArea" << markerVelY*crossSectionArea;
    }
    else{
        forceX = 0;
        forceY = 0;
        forceZ = 0;
    }

    //ChVector<> force = ChVector<>(forceX,forceY,0);
    ChVector<> force = ChVector<>(forceX, forceY, 0);
    qDebug() << "calc: damping force x: " << forceX;
    qDebug() << "calc: damping force y: " << forceY;
    qDebug() << "calc: damping force z: " << forceZ;
//    qDebug() << "damping force elem: " << force.Length();

    dampingForce->SetForce(force,false);
    dampingForce->SetApplicationPoint(marker->GetAbsCoord().pos,false);
}

void DampingElement::render(){

//    qDebug() << "render damping element";
    glLineWidth(3);

    CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
    //    qDebug()<< "markerPos.x :" << markerPos.x;
    //    qDebug()<< "markerPos.y :" << markerPos.y;
    //    qDebug()<< "markerPos.z :" << markerPos.z;

    ChVector<> force = dampingForce->GetForce();
    //ChVector<> force = monopile->getCylinder()->TransformDirectionLocalToParent(dampingForce->GetForce());

    qDebug() << " damping force x: " << force.x();
    qDebug() << " damping force y: " << force.y();
    qDebug() << " damping force z: " << force.z();

    glBegin(GL_POINTS);
        //red/light green/blue: marker
        glColor4d(1,0.5,1,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
    glEnd();
    glBegin(GL_LINES);
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

