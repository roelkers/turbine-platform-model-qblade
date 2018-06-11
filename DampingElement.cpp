
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
    ChCoordsys<> markerCoordsys = ChCoordsys<>(ChVector<>(markerPosition));
    marker->SetBody(monopile->getCylinder().get());
    marker->Impose_Abs_Coord(markerCoordsys);

    dampingForce = std::make_shared<ChLoadBodyForce> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       false, //local_force
       marker->GetPos(), //apply force at marker position
       true //local point
    );

    loadContainer->Add(dampingForce);

}

DampingElement::update(){

    marker->UpdateState();

    double markerVelX = marker->GetAbsCoord_dt().pos.x();
    double markerVelY = marker->GetAbsCoord_dt().pos.y();

    double forceX;
    double forceY;
    //check if marker is submerged
    if(marker->GetAbsCoord().pos.z() < p.seaLevel){
        forceX = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelX*crossSectionArea;
        forceY = -0.5*p.rhoWater*p.dragCoefficientCylinderLateral*markerVelY*crossSectionArea;
        qDebug() << "element submerged";
        qDebug() << "-0.5*p.rhoWater*p.dragCoefficientCylinderLateral" << -0.5*p.rhoWater*p.dragCoefficientCylinderLateral;
        qDebug() << "markerVelY*crossSectionArea" << markerVelY*crossSectionArea;
    }
    else{
        forceX = 0;
        forceY = 0;
    }

    ChVector<> force = ChVector<>(forceX,forceY,0);

    qDebug() << "damping force elem: " << force.Length();

    dampingForce->SetForce(force,true);

}

DampingElement::render(){

//    qDebug() << "render damping element";
    glLineWidth(3);

      CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
//    qDebug()<< "markerPos.x :" << markerPos.x;
//    qDebug()<< "markerPos.y :" << markerPos.y;
//    qDebug()<< "markerPos.z :" << markerPos.z;

    glPointSize(10);
    glBegin(GL_POINTS);
        //red/light green/blue: marker
        glColor4d(1,0.5,1,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
    glEnd();
    glBegin(GL_LINES);
        //light red/light green/ blue: damping force
        glColor4d(1,0.5,0.5,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
        ChVector<> force = dampingForce->GetForce();
        CVector forceVecEnd = CVecFromChVec(marker->GetAbsCoord().pos+p.cSystemFactor*force.Normalize());
        glVertex3d(forceVecEnd.x,forceVecEnd.y,forceVecEnd.z);
    glEnd();
}

