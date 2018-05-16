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

    chrono::ChVector<> gravityCenter;

public:
    Monopile(chrono::ChSystem& system, std::shared_ptr<chrono::fea::ChMesh> mesh, PlatformParams p);

    void render();
    void updateMarkers();
    double calculateGravityCenter();
    void addNacelleAndBallast(chrono::ChSystem &system, std::shared_ptr<chrono::fea::ChMesh> mesh);

    std::shared_ptr<chrono::ChMarker> getMarkerTop(){return markerTop;}
    std::shared_ptr<chrono::ChMarker> getMarkerBottom(){return markerBottom;}
    std::shared_ptr<chrono::ChBody> getCylinder(){return cylinder;}

};

#endif // MONOPILE_H
