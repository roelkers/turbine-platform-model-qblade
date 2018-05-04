#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"

#include "../XLLT/QLLTSimulation.h"

#include "MooringLine.h"
#include "Buoyancy.h"
#include "PlatformParams.h"

class PlatformModel{

  private:
    PlatformParams p;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::fea::ChNodeFEAxyzD> monopileInitNode;
    std::shared_ptr<chrono::fea::ChMesh> mesh = std::make_shared<chrono::fea::ChMesh>();
    std::shared_ptr<chrono::ChBody> monopile;
    std::shared_ptr<chrono::ChBody> nacelleBody;
    std::shared_ptr<chrono::ChBody> ballastBody;
    std::shared_ptr<Buoyancy> buoyancy;
    std::vector<MooringLine> mooringLines;

    double dT = 0.005;

  public:
    PlatformModel(QLLTSimulation *qLLTSim);
    double calculateRestPositionOfPlatform();
    double calculateGravityCenter();
    void update(double endTime);
    void render();
    void renderMonopile();
    void renderMooringLines();

};

#endif
