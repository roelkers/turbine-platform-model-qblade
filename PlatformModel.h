#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "../XLLT/QLLTSimulation.h"

#include "MooringLine.h"
#include "Buoyancy.h"
#include "PlatformParams.h"


class PlatformModel{

  private:
    std::shared_ptr<chrono::fea::ChNodeFEAxyzD> monopileInitNode;
    std::shared_ptr<chrono::ChBodyEasyCylinder> monopile;
    std::shared_ptr<Buoyancy> buoyancy;
    std::vector<MooringLine> mooringLines;
    std::vector<chrono::ChMarker> monopileFairleadMarkers;
    QLLTSimulation *qLLTSim;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::fea::ChMesh> mesh;
    PlatformParams p;
    double dT = 0.01;

  public:
    PlatformModel(QLLTSimulation *qLLTSim);
    void update(double endTime);
    void render();
    void renderMonopile();
    void renderMooringLines();
};

#endif
