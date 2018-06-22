
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "PlatformParams.h"
#include "Monopile.h"
#include "MonopileElement.h"

using namespace chrono;
using namespace chrono::fea;
Monopile::Monopile(ChSystem &system, PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer)
    :p(p)
{

    qDebug() << "creating cylinder";
    body= std::make_shared<ChBody>();
    //set cylinder to setup position
    body->SetPos(p.towerSetupPos);

    ChVector<> inertiaVector = ChVector<>(p.massMomentInertiaInRollAndPitch,p.massMomentInertiaInRollAndPitch,p.massMomentInertiaInYaw);

    body->SetMass(p.mass);
    body->SetInertiaXX(inertiaVector);

    system.Add(body);

    qDebug() << "body mass: " << body->GetMass();

    ChVector<> zAxisMonopile = body->GetFrame_COG_to_abs().GetRot().GetZaxis();

    ChVector<> vecEtoG = ChVector<>(0,0,p.distanceZfromEtoG);

    ChVector<> zStart = p.towerSetupPos - vecEtoG;

    double lengthOfElement = p.towerHeight/p.monopileNrElements;
    double crossSectionArea = 2*p.towerRadius*lengthOfElement;
    double volume = M_PI*pow(p.towerRadius,2)*lengthOfElement;

    qDebug()<< "element volume" << volume;
    qDebug()<< "lengthOfElement" << lengthOfElement;

    ChVector<> A;
    ChVector<> B;
    A = zStart;

    for(int i = 0; i<p.monopileNrElements; i++){
        //qDebug() << "adding damping element";
        B = A+lengthOfElement*zAxisMonopile;

        MonopileElement monopileElement(p,loadContainer,body,lengthOfElement,A,B,crossSectionArea,volume);
        monopileElements.push_back(monopileElement);

        A = B;
    }

    //Init Z damping force
    dragForceZBottom = std::make_shared<ChLoadBodyForce> (
      body, //body
      ChVector<>(0,0,0), //initialize
      false, //local_force
      monopileElements.at(0).getMarker()->GetPos(), //bottom marker position is attack point of force
      true //local point
    );

}

void Monopile::update(){
    for(auto &element : monopileElements){
        element.update();
    }
}

void Monopile::render(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //blue: body
        glColor4d(0,0,1,1);
        CVector monopilePos = CVecFromChVec(body->GetPos());
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
    glEnd();

    glPointSize(0.1);
    glBegin(GL_LINES);
        //Draw Coordinate system of monopile
        ChCoordsys<> monopileCoord = body->GetCoord();
        //red: x-axis
        glColor4d(1,0,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector xAxisEnd = CVecFromChVec(body->GetPos()+p.cSystemFactor*monopileCoord.rot.GetXaxis());
        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
        //green: y-axis
        glColor4d(0,1,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector yAxisEnd = CVecFromChVec(body->GetPos()+p.cSystemFactor*monopileCoord.rot.GetYaxis());
        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
        //blue: z-axis
        glColor4d(0,0,1,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector zAxisEnd = CVecFromChVec(body->GetPos()+p.cSystemFactor*monopileCoord.rot.GetZaxis());
        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);
    glEnd();

    for(auto &element : monopileElements){
        element.render();
    }
}

