
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

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

    ChVector<> markerPosition = (A + B)/2;

    marker = std::make_shared<ChMarker>();
    ChCoordsys<> markerCoordsys = ChCoordsys<>(ChVector<>(markerPosition));
    //Set marker parameters
    marker->SetBody(monopile->getCylinder().get());
    marker->Impose_Abs_Coord(markerCoordsys);

    dampingForce = std::make_shared<ChLoadBodyForce> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       false, //local_force
       ChVector<>(0,0,0), //local Gravity Center
       true //local point
    );

    loadContainer->Add(dampingForce);

}

DampingElement::update(){

    marker->UpdateState();
}

DampingElement::render(){

//    qDebug() << "render damping element";
//    glLineWidth(3);

      CVector markerPos = CVecFromChVec(marker->GetAbsCoord().pos);
//    qDebug()<< "markerPos.x :" << markerPos.x;
//    qDebug()<< "markerPos.y :" << markerPos.y;
//    qDebug()<< "markerPos.z :" << markerPos.z;

    glPointSize(10);
    glBegin(GL_POINTS);
        //red/light green/blue:
        glColor4d(1,0.5,1,1);
        glVertex3d(markerPos.x,markerPos.y,markerPos.z);
    glEnd();

}

