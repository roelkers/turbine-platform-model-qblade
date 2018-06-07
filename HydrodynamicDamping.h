#ifndef HYDRODYNAMICDAMPING_H
#define HYDRODYNAMICDAMPING_H

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "PlatformParams.h"
#include "Monopile.h"
#include "DampingElement.h"

//class DampingElement;

class HydrodynamicDamping
{
public:
    PlatformParams p;
    std::shared_ptr<Monopile> monopile;
    std::shared_ptr<chrono::ChLoadBodyForce> dragForceZBottom;
    std::vector<DampingElement> dampingElements;

    HydrodynamicDamping(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile);
    update();


    void render();

    std::shared_ptr<chrono::ChLoadBodyForce> getDragForceZBottom() const;
};

#endif // HYDRODYNAMICDAMPINGFORCE_H

