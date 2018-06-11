#include "HydrodynamicDamping.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "math.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "PlatformParams.h"
#include "Monopile.h"
#include "DampingElement.h"

using namespace chrono;
using namespace chrono::fea;


std::shared_ptr<chrono::ChLoadBodyForce> HydrodynamicDamping::getDragForceZBottom() const
{
    return dragForceZBottom;
}

HydrodynamicDamping::HydrodynamicDamping(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile)
    :p(p),
    monopile(monopile)
{
    //Init Z damping force
    dragForceZBottom = std::make_shared<ChLoadBodyForce> (
      monopile->getBallast(), //body
      ChVector<>(0,0,0), //initialize
      false, //local_force
      ChVector<>(0,0,0), //local Gravity Center
      true //local point
    );

    ChVector<> yAxisMonopile = monopile->getCylinder()->GetFrame_COG_to_abs().GetRot().GetYaxis();

    ChVector<> yStart = monopile->getBallast()->GetPos();
    //ChVector<> yEnd = monopile->getNacelle()->GetPos();

    double lengthOfElement = p.towerHeight/p.dampingNrElements;
    double crossSectionArea = 2*p.towerRadius*lengthOfElement;
    ChVector<> A;
    ChVector<> B;
    A = yStart;

    for(int i = 0; i<p.dampingNrElements; i++){
        qDebug() << "adding damping element";
        B = A+lengthOfElement*yAxisMonopile;

        DampingElement dampingElement(p,loadContainer,monopile,lengthOfElement,A,B,crossSectionArea);
        dampingElements.push_back(dampingElement);

        A = B;
    }
    //Add load to container
    loadContainer->Add(dragForceZBottom);
}

void HydrodynamicDamping::update(){

    //update drag due to force on the monopile bottom in z direction
    double speedMonopileBottomZ = monopile->getBallast()->GetPos_dt().z();
    double areaCrossSection = M_PI*pow(p.towerRadius,2);
    double forceZ = -0.5*p.rhoWater*speedMonopileBottomZ*abs(speedMonopileBottomZ)*p.dragCoefficientCylinderAxial*areaCrossSection;

    ChVector<> submergedVector = monopile->getSubmergedVector();

    //sanity check, whether monopile is actually submerged
    if(submergedVector.Length()>0){
        dragForceZBottom->SetForce(ChVector<>(0,0,forceZ),false);
    }
    else{
        dragForceZBottom->SetForce(ChVector<>(0,0,0),false);
    }

    for(auto &element : dampingElements){
        element.update();
    }
}

void HydrodynamicDamping::render(){

    for(auto &element : dampingElements){
        element.render();
    }
}
