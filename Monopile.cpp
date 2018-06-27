
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
    qDebug() << "body inertia xx" << body->GetInertiaXX().x();
    qDebug() << "body inertia yy" << body->GetInertiaXX().y();
    qDebug() << "body inertia zz" << body->GetInertiaXX().z();

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
      true, //local force
      monopileElements.at(0).getMarker()->GetPos(), //bottom marker position is attack point of force
      true //local point
    );

}

void Monopile::update(){
    //update elements
    for(auto &element : monopileElements){
        element.update();
    }

    ChVector<> markerVelocityAbs = monopileElements.at(0).getMarker()->GetAbsFrame().GetPos_dt();
    ChVector<> markerVelocityDir = body->TransformDirectionParentToLocal(markerVelocityAbs);
    //calculate drag force on the bottom end of the cylinder
    double markerVelZ = markerVelocityAbs.z();

    qDebug() << "markerVelocity z" << markerVelZ;

    //simplification since the projection of the end of the zylinder on the xy plane will actually be an ellipsis
    double bottomEndAreaXY = M_PI*pow(p.towerRadius,2);
    double dragForceZValue = 0;
    if(markerVelZ < 0){
        dragForceZValue = -0.5*p.rhoWater*p.dragCoefficientCylinderAxial*markerVelZ*bottomEndAreaXY;
    }
    else{
        dragForceZValue = 0;
    }
    //only apply damping force for a downwards motion of the platform
    dragForceZBottom->SetForce(ChVector<>(0,0,dragForceZValue),false);

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
        //Draw Damping Force on the Bottom: light green
        glLineWidth(7);
        ChVector<> bottomPos = monopileElements.at(0).getMarker()->GetAbsCoord().pos;
        CVector bottomPosCVec = CVecFromChVec(bottomPos);
        glColor4d(1,0.5,0,1);
        glVertex3d(bottomPosCVec.x,bottomPosCVec.y,bottomPosCVec.z);
        ChVector<> dragForceZBottomAbs = body->TransformDirectionLocalToParent(dragForceZBottom->GetForce());
        CVector bottomForceEnd = CVecFromChVec(bottomPos+dragForceZBottomAbs*p.forceLineFactor);
//        qDebug() << "bottom drag force z:" << dragForceZBottom->GetForce().Length();
//        qDebug() << "bottomForceEnd x " << bottomForceEnd.x;
//        qDebug() << "bottomForceEnd y " << bottomForceEnd.y;
//        qDebug() << "bottomForceEnd z " << bottomForceEnd.z;
//        qDebug() << "bottomPos x" << bottomPos.x();
//        qDebug() << "bottomPos y" << bottomPos.y();
//        qDebug() << "bottomPos z" << bottomPos.z();
        glVertex3d(bottomForceEnd.x,bottomForceEnd.y,bottomForceEnd.z);
    glEnd();
    //Render Elements
    for(auto &element : monopileElements){
        element.render();
    }
}

