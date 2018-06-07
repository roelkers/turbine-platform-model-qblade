#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"

#include "../XLLT/QLLTSimulation.h"

#include "Monopile.h"
#include "MooringLine.h"
#include "Buoyancy.h"
#include "PlatformParams.h"
#include "HydrodynamicDamping.h"

class HydrodynamicDamping;

class PlatformModel{

  private:
    PlatformParams p;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::fea::ChNodeFEAxyzD> monopileInitNode;
    std::shared_ptr<chrono::fea::ChMesh> mesh = std::make_shared<chrono::fea::ChMesh>();
    std::shared_ptr<Monopile> monopile;
    std::shared_ptr<Buoyancy> buoyancy;
    std::shared_ptr<HydrodynamicDamping> hydrodynamicDamping;
    std::vector<MooringLine> mooringLines;

//    double dT = 0.005;
    QLLTSimulation *m_qlltSim;
  public:
    PlatformModel(QLLTSimulation *qLLTSim);
    double calculateRestPositionOfPlatform();
    void update(double endTime);
    void render();
    double getXPosition();
    double getYPosition();
    double getZPosition();
    double getRollAngle();
    double getPitchAngle();
    double getYawAngle();
    double getDragForceZBottom();

};

#endif
