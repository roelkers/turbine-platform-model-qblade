
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
Monopile::Monopile(ChSystem &system, PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer)
    :p(p)
{

    qDebug() << "creating cylinder";
    body= std::make_shared<ChBody>();
    //set cylinder to setup position
    body->SetPos(p.towerSetupPos);

    ChVector<> inertiaVector = ChVector<>(p.massMomentInertiaInRollAndPitch,p.massMomentInertiaInRollAndPitch,p.massMomentInertiaInYaw);

    body->setMass(p.mass);
    body->SetInertiaXX(inertiaVector);

    system.Add(cylinder);

    qDebug() << "body mass: " << body->GetMass();

//    ChVector<> yAxisMonopile = monopile->getBody()->GetFrame_COG_to_abs().GetRot().GetYaxis();

    ChVector<> zStart = monopile->getBallast()->GetPos();
    //ChVector<> yEnd = monopile->getNacelle()->GetPos();

    double lengthOfElement = p.towerHeight/p.dampingNrElements;
    double crossSectionArea = 2*p.towerRadius*lengthOfElement;
    double volume = M_PI*pow(p.towerRadius,2)*lengthOfElement;

    ChVector<> A;
    ChVector<> B;
    A = zStart;

    for(int i = 0; i<p.monopileNrElements; i++){
        //qDebug() << "adding damping element";
        B = A+lengthOfElement*zAxisMonopile;

        monopileElement dampingElement(p,loadContainer,monopile,lengthOfElement,A,B,crossSectionArea,volume);
        monopileElements.push_back(monopileElement);

        A = B;
    }

}

void Monopile::update(){

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

