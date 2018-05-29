#ifndef HYDRODYNAMICDAMPING_H
#define HYDRODYNAMICDAMPING_H

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "PlatformParams.h"
#include "Monopile.h"

class HydrodynamicDamping
{
public:
    PlatformParams p;
    std::shared_ptr<chrono::ChLoadBodyForce> dragForceZBottom;
    std::shared_ptr<chrono::ChLoadBodyForce> dragForceXY;
    std::shared_ptr<Monopile> monopile;

    HydrodynamicDamping(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile);
    update();

};

#endif // HYDRODYNAMICDAMPINGFORCE_H

