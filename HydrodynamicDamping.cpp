#include "HydrodynamicDamping.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "math.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "PlatformParams.h"
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

std::shared_ptr<chrono::ChLoadBodyTorque> HydrodynamicDamping::getDragTorqueX() const
{
    return dragTorqueX;
}

std::shared_ptr<chrono::ChLoadBodyForce> HydrodynamicDamping::getDragForceZBottom() const
{
    return dragForceZBottom;
}

std::shared_ptr<chrono::ChLoadBodyForce> HydrodynamicDamping::getDragForceXY() const
{
    return dragForceXY;
}

std::shared_ptr<chrono::ChLoadBodyTorque> HydrodynamicDamping::getDragTorqueZ() const
{
    return dragTorqueZ;
}

chrono::ChVector<> HydrodynamicDamping::getPhiDot() const
{
    return phiDot;
}

HydrodynamicDamping::HydrodynamicDamping(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile)
    :p(p),
    monopile(monopile)
{
    //Init damping force with null vectors
    dragForceZBottom = std::make_shared<ChLoadBodyForce> (
      monopile->getBallast(), //body
      ChVector<>(0,0,0), //initialize
      false, //local_force
      ChVector<>(0,0,0), //local Gravity Center
      true //local point
    );

    dragForceXY = std::make_shared<ChLoadBodyForce> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       false, //local_force
       ChVector<>(0,0,0), //local Gravity Center
       true //local point
    );

    dragTorqueX = std::make_shared<ChLoadBodyTorque> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       true //local_force
    );

    dragTorqueZ = std::make_shared<ChLoadBodyTorque> (
       monopile->getCylinder(), //body
       ChVector<>(0,0,0), //initialize
       true //local_force
    );

    //Add load to container
    loadContainer->Add(dragForceZBottom);
    loadContainer->Add(dragForceXY);
    //loadContainer->Add(dragTorqueX);
    //loadContainer->Add(dragTorqueZ);
}

HydrodynamicDamping::update(){

    //update drag due to force on the monopile bottom in z direction
    double speedMonopileBottomZ = monopile->getBallast()->GetPos_dt().z();
    double areaCrossSection = M_PI*pow(p.towerRadius,2);
    double forceZ = -0.5*p.rhoWater*speedMonopileBottomZ*abs(speedMonopileBottomZ)*p.dragCoefficientCylinderAxial*areaCrossSection;

    //update drag in x and y direction on the monopile, this force is applied at the buoyancy center
    double speedMonopileX = monopile->getCylinder()->GetPos_dt().x();
    double speedMonopileY = monopile->getCylinder()->GetPos_dt().y();

    ChVector<> submergedVector = monopile->getSubmergedVector();

    double submergedLengthYZ = sqrt(pow(submergedVector.y(),2)+pow(submergedVector.z(),2));
    double areaYZSurface = submergedLengthYZ*2*p.towerRadius;
    double forceX = -0.5*p.rhoWater*speedMonopileX*abs(speedMonopileX)*p.dragCoefficientCylinderLateral*areaYZSurface;

    double submergedLengthXZ = sqrt(pow(submergedVector.x(),2)+pow(submergedVector.z(),2));
    double areaXZSurface = submergedLengthXZ*2*p.towerRadius;
    double forceY = -0.5*p.rhoWater*speedMonopileY*abs(speedMonopileY)*p.dragCoefficientCylinderLateral*areaXZSurface;

//    qDebug() << "speedMonopileX:" << speedMonopileX;
//    qDebug() << "speedMonopileY:" << speedMonopileY;

//    qDebug() << "areaXZSurface: " << areaXZSurface;
//    qDebug() << "areaYZSurface: " << areaYZSurface;

//    qDebug() << "hydrodynamic drag force x:" << forceX;
//    qDebug() << "hydrodynamic drag force y:" << forceY;
//    qDebug() << "hydrodynamic drag force z:" << forceZ;

    //update drag torque in local x and z direction (torque around local body x and z axis)

    double torqueX = 0;
    double torqueZ = 0;

    ChFrameMoving<> frameMoving = monopile->getCylinder()->GetFrame_COG_to_abs();

    //Get Angular Speeds in local body coordinates
    phiDot = frameMoving.GetWvel_loc();

    qDebug()<<  "submerged Part of Monopile:" << monopile->submergedPart;

    ChVector<> vecG = monopile->getCylinder()->GetPos();
    ChVector<> vecS = monopile->getIntersectionPoint();
    ChVector<> vecE = monopile->getBallast()->GetPos();
    ChVector<> vecI = monopile->getNacelle()->GetPos();
    //vector from gravity center to intersection point
    ChVector<> vecGS = vecS - vecG;
    //vector from gravity center to ballast
    ChVector<> vecGE = vecE - vecG;
    //vector from gravity center to interface
    ChVector<> vecGI = vecI - vecG;

    switch(monopile->submergedPart){
        case Monopile::BALLAST:{

        if(vecG.x()>p.seaLevel){
            //integrate from point S to bottom if G is above sea level
            torqueX = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*(pow(vecGE.Length(),4)-pow(vecGS.Length(),4));
            torqueZ = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*(pow(vecGE.Length(),4)-pow(vecGS.Length(),4));
        }
        else{
            //integrate from Gravity centre G to bottom and to sea level respectively otherwise
            torqueX = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*(pow(vecGS.Length(),4)+pow(vecGE.Length(),4));
            torqueZ = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*(pow(vecGS.Length(),4)+pow(vecGE.Length(),4));
        }
        qDebug() << "pow(vecGS.Length(),4)+pow(vecGE.Length(),4)" << pow(vecGS.Length(),4)+pow(vecGE.Length(),4);
        qDebug() << "pow(vecGE.Length(),4)-pow(vecGS.Length(),4)" << pow(vecGE.Length(),4)-pow(vecGS.Length(),4);

        break;
        }
        case Monopile::NACELLE:{

        if(vecG.x()>p.seaLevel){
            //integrate from point S to bottom if G is above sea level
            torqueX = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*(pow(vecGI.Length(),4)-pow(vecGS.Length(),4));
            torqueZ = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*(pow(vecGI.Length(),4)-pow(vecGS.Length(),4));
        }
        else{
            //integrate from Gravity centre G to bottom and to sea level respectively otherwise
            torqueX = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*(pow(vecGS.Length(),4)+pow(vecGI.Length(),4));
            torqueZ = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*(pow(vecGS.Length(),4)+pow(vecGI.Length(),4));
        }
        qDebug() << "pow(vecGS.Length(),4)+pow(vecGI.Length(),4)" << pow(vecGS.Length(),4)+pow(vecGI.Length(),4);
        qDebug() << "pow(vecGS.Length(),4)+pow(vecGI.Length(),4)" << pow(vecGI.Length(),4)-pow(vecGS.Length(),4);
        break;
        }
        case Monopile::BOTH:{

        //integrate from Gravity centre G to bottom and to top respectively
        torqueX = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*(pow(vecGI.Length(),4)+pow(vecGE.Length(),4));
        torqueZ = -0.25*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*(pow(vecGI.Length(),4)+pow(vecGE.Length(),4));
        qDebug() << "pow(vecGI.Length(),4)+pow(vecGE.Length(),4)" << pow(vecGI.Length(),4)+pow(vecGE.Length(),4),
        qDebug() << "pow(vecGI.Length(),4)+pow(vecGE.Length(),4)" << pow(vecGI.Length(),4)+pow(vecGE.Length(),4);
        break;
        }
        case Monopile::NONE:{

        torqueX = 0;
        torqueZ = 0;

        break;
        }
    }

    //get local x and z axis for torque application
    ChVector<> xAxisLocal = frameMoving.GetCoord().rot.GetXaxis();
    ChVector<> zAxisLocal = frameMoving.GetCoord().rot.GetZaxis();

    qDebug() << "vecGI.Length()" << vecGI.Length();
    qDebug() << "vecGE.Length()" << vecGI.Length();
    qDebug() << "vecG.Length()" << vecG.Length();
    qDebug() << "vecE.Length()" << vecE.Length();
    qDebug() << "vecI.Length()" << vecI.Length();
    qDebug() << "pow(phiDot.x(),2)" << pow(phiDot.x(),2);
    qDebug() << "pow(phiDot.z(),2)" << pow(phiDot.z(),2);
    qDebug() << "torqueX: " << torqueX;
    qDebug() << "torqueZ: " << torqueZ;

    //drag torque around local y axis is zero for the cylinder (for a rotation around its own axis a cylinder does not displace any fluid, there is only friction)
    //therefore there is no need to set torque around local y axis

    dragForceXY->SetApplicationPoint(monopile->getBuoyancyCenter(),false);

    //sanity check, whether monopile is actually submerged
    if(submergedVector.Length()>0){
        dragForceZBottom->SetForce(ChVector<>(0,0,forceZ),false);
        dragForceXY->SetForce(ChVector<>(forceX,forceY,0),false);
        dragTorqueX->SetTorque(torqueX*xAxisLocal);
        dragTorqueZ->SetTorque(torqueZ*zAxisLocal);
    }
    else{
        dragForceZBottom->SetForce(ChVector<>(0,0,0),false);
        dragForceXY->SetForce(ChVector<>(0,0,0),false);
        dragTorqueX->SetTorque(ChVector<>(0,0,0));
        dragTorqueZ->SetTorque(ChVector<>(0,0,0));
    }
}

void HydrodynamicDamping::render(){
    glPointSize(10);
    glLineWidth(3);
    CVector monopilePos = CVecFromChVec(monopile->getCylinder()->GetPos());

    ChVector<> dragTorqueXVec = dragTorqueX->GetTorque();
    ChVector<> dragTorqueZVec = dragTorqueZ->GetTorque();

    glPointSize(0.1);
    glBegin(GL_LINES);
        //light green/blue: x-torque
        glColor4d(0,0.5,1,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector torqueAxisEnd = CVecFromChVec(monopile->getCylinder()->GetPos()+dragTorqueXVec);
        glVertex3d(torqueAxisEnd.x,torqueAxisEnd.y,torqueAxisEnd.z);
        //blue /light green: z-torque
        glColor4d(0,1,0.5,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        torqueAxisEnd = CVecFromChVec(monopile->getCylinder()->GetPos()+dragTorqueZVec);
        glVertex3d(torqueAxisEnd.x,torqueAxisEnd.y,torqueAxisEnd.z);
    glEnd();
}
