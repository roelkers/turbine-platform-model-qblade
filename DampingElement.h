#ifndef DAMPINGELEMENT_H
#define DAMPINGELEMENT_H

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "Monopile.h"
#include "PlatformParams.h"

class DampingElement
{
private:

    PlatformParams p;
    double length;
    double crossSectionArea;
    std::shared_ptr<Monopile> monopile;
    chrono::ChVector<> A; //start of element
    chrono::ChVector<> B; //end of element
    std::shared_ptr<chrono::ChMarker> marker;

    std::shared_ptr<chrono::ChLoadBodyForce> dampingForce;

public:
    DampingElement(PlatformParams p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<Monopile> monopile, double length, chrono::ChVector<> A, chrono::ChVector<> B, double crossSectionArea);
    update();
    render();
};

#endif // DAMPINGELEMENT_H
