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
    dampingForce = std::make_shared<ChLoadBodyForce> (
      monopile->getBallast(), //body
      ChVector<>(0,0,0), //force in positive z direction
      false, //local_force
      ChVector<>(0,0,0), //local Gravity Center
      true //local point
    );

    //Add load to container
    loadContainer->Add(dampingForce);
}

HydrodynamicDamping::update(){

    double speedMonopileBottomZ = monopile->getBallast()->GetPos_dt().z();

    double force = -0.5*p.rhoWater*speedMonopileBottomZ*abs(speedMonopileBottomZ)*M_PI*p.towerRadius;

    dampingForce->SetForce(ChVector<>(0,0,force),false);
}


