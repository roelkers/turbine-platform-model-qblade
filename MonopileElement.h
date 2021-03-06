#ifndef MONOPILEELEMENT_H
#define MONOPILEELEMENT_H

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "Monopile.h"
#include "PlatformParams.h"

class Monopile;

class MonopileElement
{
private:

    PlatformParams p;
    double length;
    double crossSectionArea;
    double volume;

    chrono::ChVector<> waveVelocityLocalVec;
    double waveVelocity;
    chrono::ChVector<> waveAccelerationLocalVec;
    double waveAcceleration;

    chrono::ChVector<> markerAccDir;

    std::shared_ptr<chrono::ChBody> body;
    chrono::ChVector<> A; //start of element
    chrono::ChVector<> B; //end of element

    chrono::ChVector<> AinAbsoluteFrame;
    chrono::ChVector<> BinAbsoluteFrame;

    std::shared_ptr<chrono::ChMarker> marker;

    std::shared_ptr<chrono::ChLoadBodyForce> dragForce;
    std::shared_ptr<chrono::ChLoadBodyForce> buoyancyForce;
    std::shared_ptr<chrono::ChLoadBodyForce> addedDampingForce;

public:
    MonopileElement(PlatformParams &p, std::shared_ptr<chrono::ChLoadContainer> loadContainer, std::shared_ptr<chrono::ChBody> body, double length, chrono::ChVector<> const& A, chrono::ChVector<> const& B, double crossSectionArea, double volume);
    void update(double const seaLevel, double const time);
    void render() const;
    std::shared_ptr<chrono::ChMarker> getMarker() const;
    bool isSubmerged(double seaLevel) const;
};

#endif // MONOPILEELEMENT_H
