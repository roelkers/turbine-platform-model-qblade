#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "PlatformModel.h"
#include "PlatformParams.h"
#include "Buoyancy.h"

using namespace chrono;
using namespace chrono::fea;

PlatformModel::PlatformModel(QLLTSimulation *qLLTSim)
    :p(qLLTSim->getTowerHeight())
{

    qDebug() << "tower height: " << p.towerHeight;
    // Change solver settings
    system.SetSolverType(ChSolver::Type::MINRES);
    system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    system.SetMaxItersSolverSpeed(200);
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto solver = std::static_pointer_cast<ChSolverMINRES>(system.GetSolver());
    solver->SetVerbose(false);
    solver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    mesh = std::make_shared<ChMesh>();

    //Monopile
    monopileInitNode = std::make_shared<ChNodeFEAxyzD>(p.towerInitPos, p.towerInitDir);
    mesh->AddNode(monopileInitNode);

    monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
    monopile->SetPos(monopileInitNode->GetPos());

    //Setup location of monopile for construction of mooring lines
    ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);

    monopile->SetRot(qSetup);

    system.Add(monopile);

    //Constraints

    //Add Buoyancy force

    //Init Load container
    auto loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    buoyancy = std::make_shared<Buoyancy>(p, loadcontainer, monopile, mesh, system);

    //Add Gravity
    system.Set_G_acc(ChVector<>(0,0,-p.g));
    //system.Set_G_acc(ChVector<>(0,0,0));

    //Angular increment of Mooring Line on Monopile
    double thetaInc = 360/p.mooringLineNr;
    //Angle on Monopile of mooring line
    double theta = 0;
    //Construct Mooring Lines
    for(int i = 0; i < p.mooringLineNr; i++){
        theta = theta + thetaInc;

        qDebug() << "Mooring Line Angular position: " << theta << "deg\n";
        qDebug() << "Constructing mooring line " << i << "\n";
        MooringLine mLine(system, mesh, p, theta, monopile);
        mooringLines.push_back(mLine);

    }
    //Initial rotation of the monopile
    //Rotate around x axis and y axis
    ChQuaternion<> qRotationX = Q_from_AngAxis(10 * CH_C_DEG_TO_RAD, VECT_X);
    ChQuaternion<> qRotationZ= Q_from_AngAxis(20 * CH_C_DEG_TO_RAD, VECT_Z);
    //Translate to initial Position
    ChVector<> initPos = ChVector<>(0,0,0.5*p.towerHeight);
    //ChVector<> initPos = ChVector<>(20,20,0);

    //Define initial displacement
    ChCoordsys<> initCoords =ChCoordsys<>(initPos,qRotationX*qRotationZ);
    monopile->Move(initCoords);

    //qDebug() << "monopile initial position:" << monopile->GetPos() << "\n";
    //qDebug() << "monopile initial rotation:" << monopile->GetRot() << "\n";
    qDebug() << "Rest Length: " << p.mooringRestLength << "\n";

    system.Add(mesh);
    system.SetupInitial();

}

void PlatformModel::render(){

    glNewList(GLPLATFORM,GL_COMPILE);
    {
        renderMonopile();
        renderMooringLines();
    }
    glEndList();
}

void PlatformModel::renderMonopile(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //blue: monopile
        glColor4d(0,0,1,1);
        CVector monopilePos = CVecFromChVec(monopile->GetPos());
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        //green: topMarker
        glColor4d(0,1,0,1);
        CVector topMarkerPos = CVecFromChVec(buoyancy->getMarkerTop()->GetAbsCoord().pos);
        glVertex3d(topMarkerPos.x,topMarkerPos.y,topMarkerPos.z);
        //red: bottomMarker
        glColor4d(1,0,0,1);
        CVector bottomMarkerPos = CVecFromChVec(buoyancy->getMarkerBottom()->GetAbsCoord().pos);
        glVertex3d(bottomMarkerPos.x,bottomMarkerPos.y,bottomMarkerPos.z);
        //red/green: intersection Point
        glColor4d(1,1,0,1);
        CVector isPos = CVecFromChVec(buoyancy->getIntersectionPoint());
        glVertex3d(isPos.x,isPos.y,isPos.z);
        //blue/green: buoyancy center
        glColor4d(0,1,1,1);
        CVector buoyancyCenterPos = CVecFromChVec(buoyancy->getBuoyancyCenter());
        glVertex3d(buoyancyCenterPos.x,buoyancyCenterPos.y,buoyancyCenterPos.z);
    glEnd();
    //Draw axis of tower
    glPointSize(0.1);
    glBegin(GL_LINES);
        glColor4d(0,0,0,1);
        glVertex3d(topMarkerPos.x,topMarkerPos.y,topMarkerPos.z);
        glVertex3d(bottomMarkerPos.x,bottomMarkerPos.y,bottomMarkerPos.z);
    glEnd();
}

void PlatformModel::renderMooringLines(){
   glPointSize(0.1);
   glLineWidth(1);

   for(auto & mooringLine : mooringLines) {
       mooringLine.render();
   }
}

void PlatformModel::update(double endTime){

    double old_step;
    double left_time;
    int restore_oldstep = FALSE;

    system.SetStep(dT);

    while (system.GetChTime() < endTime) {

        buoyancy->update();

        restore_oldstep = FALSE;
        left_time = endTime - system.GetChTime();
        if (left_time < 1e-12) break;  // - no integration if backward or null frame step.
        if (left_time < (1.3 * dT))  // - step changed if too little frame step
        {
            old_step = dT;
            dT = left_time;
            restore_oldstep = TRUE;
        }
        if (!system.DoStepDynamics(dT)) break;  // ***  Single integration step,

    }

    if (restore_oldstep) dT = old_step;
}
