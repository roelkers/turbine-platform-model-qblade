#ifndef MONOPILE_H
#define MONOPILE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/core/ChQuaternion.h"
#include "../XLLT/QLLTSimulation.h"

#include "PlatformParams.h"
#include "MonopileElement.h"

class Monopile
{

private:
    PlatformParams p;

    std::shared_ptr<chrono::ChBody> platformBody;
    std::shared_ptr<chrono::ChBody> nacelleBody;
    std::shared_ptr<chrono::ChBody> hubBody;
    std::shared_ptr<chrono::ChBody> towerBody;
    std::shared_ptr<chrono::ChBody> interfaceBody;
    std::vector<std::shared_ptr<chrono::ChBody>> bladeBodies;

    std::vector<MonopileElement> monopileElements;
    std::shared_ptr<chrono::ChLoadBodyForce> addedDampingForce;
    std::shared_ptr<chrono::ChLoadBodyTorque> addedYawSpringTorque;
    std::shared_ptr<chrono::ChLoadBodyTorque> addedYawDampingTorque;

    std::shared_ptr<chrono::ChLoadBodyForce> aerolasticInterfaceForce;
    std::shared_ptr<chrono::ChLoadBodyTorque> aerolasticInterfaceTorque;

public:
    Monopile(chrono::ChSystem& system, PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer);

    void render();
    void update(chrono::ChVector<> force, chrono::ChVector<> torque, double seaLevel, double time);
    void addMasses(chrono::ChSystem &system);

    std::shared_ptr<chrono::ChBody> getBody(){return platformBody;}
    CVector getInterfacePos();
    chrono::ChQuaternion<> getInterfaceRot();

    //std::shared_ptr<chrono::ChLoadBodyForce> getAddedDampingForce() const;
};

#endif // MONOPILE_H
