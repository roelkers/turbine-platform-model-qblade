#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/assets/ChTexture.h"

#include <QDebug>
#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "Buoyancy.h"
#include "PlatformParams.h"
#include "Monopile.h"

using namespace chrono;
using namespace chrono::fea;

Buoyancy::Buoyancy(PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile)

:p(p),
monopile(monopile),
loadContainer(loadContainer)
{

  //Init Buoyancy force with null vectors
  buoyancyForce = std::make_shared<ChLoadBodyForce> (
    monopile->getCylinder(), //body
    ChVector<>(0,0,0), //force in positive z direction
    false, //local_force
    ChVector<>(0,0,0), //local Gravity Center
    true //local point
  );

  //Update Buoyancy Force
  update();

  //Add load to container
  loadContainer->Add(buoyancyForce);
}


void Buoyancy::update(){

  ChVector<> intersectionPoint = monopile->getIntersectionPoint();

  ChVector<> vecE = monopile->getBallast()->GetPos();
  ChVector<> vecI = monopile->getNacelle()->GetPos();

  ChVector<> submergedVector = monopile->getSubmergedVector();
  double force = computeBuoyancyForce(submergedVector.Length());

  //qDebug() << "submerged Part of Monopile:" << monopile->submergedPart;
  qDebug() << "buoyancyForce:" << force << "\n";

  monopile->setBuoyancyCenter(monopile->getBuoyancyCenter());

  buoyancyForce->SetApplicationPoint(monopile->getBuoyancyCenter(),false);

  buoyancyForce->SetForce(ChVector<>(0,0,force),false);


}

double Buoyancy::computeBuoyancyForce(double submergedLength){
    double submergedVolumeMonopile = M_PI*pow(p.towerRadius,2)*submergedLength;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}

void Buoyancy::render(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //red/green: intersection Point
        glColor4d(1,1,0,1);
        CVector isPos = CVecFromChVec(monopile->getIntersectionPoint());
        glVertex3d(isPos.x,isPos.y,isPos.z);
        //blue/green: buoyancy center
        glColor4d(0,1,1,1);
        CVector buoyancyCenterPos = CVecFromChVec(monopile->getBuoyancyCenter());
        glVertex3d(buoyancyCenterPos.x,buoyancyCenterPos.y,buoyancyCenterPos.z);
    glEnd();
}
