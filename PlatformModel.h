#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"

#include "../XLLT/QLLTSimulation.h"

#include "Monopile.h"
#include "MooringLine.h"
#include "PlatformParams.h"

class HydrodynamicDamping;

class PlatformModel{

  private:
    PlatformParams p;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::fea::ChMesh> mesh = std::make_shared<chrono::fea::ChMesh>();
    std::shared_ptr<Monopile> monopile;
    std::vector<MooringLine> mooringLines;

    QLLTSimulation *m_qlltSim;
  public:
    PlatformModel(QLLTSimulation *qLLTSim);
    double calculateRestPositionOfPlatform();
    void update(double endTime, CVector aerolasticInterfaceForce, CVector aerolasticInterfaceTorque);
    void render();
    double getXPosition();
    double getYPosition();
    double getZPosition();
    double getRollAngle();
    double getPitchAngle();
    double getYawAngle();
    //double getDragForceZBottom();
    double getDampingForceZ();
    double getVelocityZ();
    CVector getInterfacePos();
};

#endif
