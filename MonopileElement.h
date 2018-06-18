#ifndef MONOPILEELEMENT_H
#define MONOPILEELEMENT_H

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "Monopile.h"
#include "PlatformParams.h"

class MonopileElement
{
private:

    PlatformParams p;
    double length;
    double crossSectionArea;

    std::shared_ptr<Monopile> monopile;
    chrono::ChVector<> A; //start of element
    chrono::ChVector<> B; //end of element

    chrono::ChVector<> AinAbsoluteFrame;
    chrono::ChVector<> BinAbsoluteFrame;

    std::shared_ptr<chrono::ChMarker> marker;

    std::shared_ptr<chrono::ChLoadBodyForce> dampingForce;
    std::shared_ptr<chrono::ChLoadBodyForce> buoyancyForce;
public:
    MonopileElement(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile, double length, chrono::ChVector<> A, chrono::ChVector<> B, double crossSectionArea);
    void update();
    void render();
};

#endif // MONOPILEELEMENT_H
