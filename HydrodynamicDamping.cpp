#include "HydrodynamicDamping.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "math.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "PlatformParams.h"
#include "Monopile.h"
#include "MonopileElement.h"

using namespace chrono;
using namespace chrono::fea;


std::shared_ptr<chrono::ChLoadBodyForce> HydrodynamicDamping::getDragForceZBottom() const
{
    return dragForceZBottom;
}

HydrodynamicDamping::HydrodynamicDamping(PlatformParams p, std::shared_ptr<Monopile> monopile)
    :p(p),
    monopile(monopile)
{

}

void HydrodynamicDamping::update(){

//    //update drag due to force on the monopile bottom in z direction
//    double speedMonopileBottomZ = monopile->getBallast()->GetPos_dt().z();
//    double areaCrossSection = M_PI*pow(p.towerRadius,2);
//    double forceZ = -0.5*p.rhoWater*speedMonopileBottomZ*abs(speedMonopileBottomZ)*p.dragCoefficientCylinderAxial*areaCrossSection;

//    ChVector<> submergedVector = monopile->getSubmergedVector();

//    //sanity check, whether monopile is actually submerged
//    if(submergedVector.Length()>0){
//        dragForceZBottom->SetForce(ChVector<>(0,0,forceZ),false);
//    }
//    else{
//        dragForceZBottom->SetForce(ChVector<>(0,0,0),false);
//    }

//    for(auto &element : dampingElements){
//        element.update();
//    }
}

void HydrodynamicDamping::render(){

//    for(auto &element : dampingElements){
//        element.render();
//    }
}
