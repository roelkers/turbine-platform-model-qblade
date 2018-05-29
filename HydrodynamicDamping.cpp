#include "HydrodynamicDamping.h"


#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "math.h"

#include "PlatformParams.h"
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

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
    loadContainer->Add(dragTorqueX);
    loadContainer->Add(dragTorqueZ);
}

HydrodynamicDamping::update(){

    //update drag due to force on the monopile bottom in z direction
    double speedMonopileBottomZ = monopile->getBallast()->GetPos_dt().z();
    double areaCrossSection = M_PI*pow(p.towerRadius,2);
    double forceZ = -0.5*p.rhoWater*speedMonopileBottomZ*abs(speedMonopileBottomZ)*areaCrossSection;

    //update drag in x and y direction on the monopile, this force is applied at the buoyancy center
    double speedMonopileX = monopile->getCylinder()->GetPos_dt().x();
    double speedMonopileY = monopile->getCylinder()->GetPos_dt().y();

    ChVector<> submergedVector = monopile->getSubmergedVector();

    double submergedLengthYZ = sqrt(pow(submergedVector.y(),2)+pow(submergedVector.z(),2));
    double areaYZSurface = submergedLengthYZ*2*p.towerRadius;
    double forceX = -0.5*p.rhoWater*speedMonopileX*abs(speedMonopileX)*areaYZSurface;

    double submergedLengthXZ = sqrt(pow(submergedVector.x(),2)+pow(submergedVector.z(),2));
    double areaXZSurface = submergedLengthXZ*2*p.towerRadius;
    double forceY = -0.5*p.rhoWater*speedMonopileY*abs(speedMonopileY)*areaXZSurface;

    qDebug() << "speedMonopileX:" << speedMonopileX;
    qDebug() << "speedMonopileY:" << speedMonopileY;

    qDebug() << "areaXZSurface: " << areaXZSurface;
    qDebug() << "areaYZSurface: " << areaYZSurface;

    qDebug() << "hydrodynamic drag force x:" << forceX;
    qDebug() << "hydrodynamic drag force y:" << forceY;
    qDebug() << "hydrodynamic drag force z:" << forceZ;

    //update drag torque in local x and z direction (torque around local body x and z axis)

    ChFrameMoving<> frameMoving = monopile->getCylinder()->GetFrame_COG_to_abs();

    //Get Angular Speeds in local body coordinates
    ChVector<> phiDot = frameMoving.GetWvel_loc();

    //analytic formula for torque:
    double torqueX = 1/4*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.x(),2)*pow(submergedVector.Length(),4);
    double torqueZ = 1/4*p.towerRadius*p.rhoWater*p.dragCoefficientCylinderLateral*pow(phiDot.z(),2)*pow(submergedVector.Length(),4);
    //drag torque around local y axis is zero for the cylinder (for a rotation around its own axis a cylinder does not displace any fluid, there is only friction)

    //get local x and z axis for torque application
    ChVector<> xAxisLocal = frameMoving.GetCoord().rot.GetXaxis();
    ChVector<> zAxisLocal = frameMoving.GetCoord().rot.GetZaxis();

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
