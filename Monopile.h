#ifndef MONOPILE_H
#define MONOPILE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"

#include "../XLLT/QLLTSimulation.h"

#include "PlatformParams.h"

class Monopile
{

private:
    PlatformParams p;

    std::shared_ptr<chrono::ChBody> body;
    std::vector<MonopileElement> monopileElements;

public:
    Monopile(chrono::ChSystem& system, PlatformParams p);

    void render();
    void update();

    std::shared_ptr<chrono::ChBody> getBody(){return body;}

};

#endif // MONOPILE_H
