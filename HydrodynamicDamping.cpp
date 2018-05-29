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

    //Add load to container
    loadContainer->Add(dragForceZBottom);
    loadContainer->Add(dragForceXY);
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

    if(submergedVector.Length()>0){
        dragForceZBottom->SetForce(ChVector<>(0,0,forceZ),false);
        dragForceXY->SetForce(ChVector<>(forceX,forceY,0),false);
    }
    else{
        dragForceZBottom->SetForce(ChVector<>(0,0,0),false);
        dragForceXY->SetForce(ChVector<>(0,0,0),false);
    }
}


