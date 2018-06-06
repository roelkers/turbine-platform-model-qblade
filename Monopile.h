#ifndef MONOPILE_H
#define MONOPILE_H

#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"

#include "../XLLT/QLLTSimulation.h"

#include "PlatformParams.h"

class Monopile
{

private:
    PlatformParams p;

    std::shared_ptr<chrono::ChBody> cylinder;
    std::shared_ptr<chrono::ChBody> nacelle;
    std::shared_ptr<chrono::ChBody> ballast;

    std::shared_ptr<chrono::ChMarker> markerBottom;
    std::shared_ptr<chrono::ChMarker> markerTop;
    std::shared_ptr<chrono::ChMarker> markerGravityCenter;

    chrono::ChVector<> buoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
    chrono::ChVector<> intersectionPoint;
    chrono::ChVector<> submergedVector;


public:
    Monopile(chrono::ChSystem& system, PlatformParams p);

    void render();
    void updateMarkers();
    chrono::ChVector<> calculateGravityCenterFromBottom();
    void addNacelleAndBallast(chrono::ChSystem &system);

    std::shared_ptr<chrono::ChMarker> getMarkerTop(){return markerTop;}
    std::shared_ptr<chrono::ChMarker> getMarkerBottom(){return markerBottom;}
    std::shared_ptr<chrono::ChBody> getCylinder(){return cylinder;}
    std::shared_ptr<chrono::ChBody> getBallast(){return ballast;}
    std::shared_ptr<chrono::ChBody> getNacelle(){return nacelle;}
    chrono::ChVector<> getBuoyancyCenter() const;
    chrono::ChVector<> getIntersectionPoint() const;
    chrono::ChVector<> getSubmergedVector() const;

    void setBuoyancyCenter(chrono::ChVector<> buoyancyCenter){this->buoyancyCenter=buoyancyCenter;}
    void setIntersectionPoint(chrono::ChVector<> intersectionPoint){this->intersectionPoint=intersectionPoint;}
    void setSubmergedVector(const chrono::ChVector<> &value);

    enum submergedPartType{
        BALLAST,
        NACELLE,
        BOTH,
        NONE
    };

    submergedPartType submergedPart;

};

#endif // MONOPILE_H
