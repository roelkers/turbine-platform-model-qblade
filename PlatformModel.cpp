#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/physics/ChLinkMate.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChMesh.h"

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
    Reset();

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

    qDebug() << "creating monopile";
    //Monopile
    monopileInitNode = std::make_shared<ChNodeFEAxyzD>(p.towerSetupPos, p.towerSetupDir);
    mesh->AddNode(monopileInitNode);
    monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
    monopile->SetPos(monopileInitNode->GetPos());

    qDebug() << "rotate monopile";
    //Setup location of monopile
    ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
    monopile->SetRot(qSetup);
    system.Add(monopile);

    qDebug() << "monopile mass: " << monopile->GetMass();

    //Create ballast on the bottom of the monopile

    std::shared_ptr<chrono::ChBody> ballastBody = std::make_shared<chrono::ChBody>();
    ballastBody->SetMass(p.ballastMass);
    system.Add(ballastBody);
    ChVector<> ballastPos = ChVector<>(0,0,0);
    ballastNode = std::make_shared<ChNodeFEAxyzD>(ballastPos, p.towerSetupDir);
    mesh->AddNode(ballastNode);
    ballastBody->SetPos(ballastNode->GetPos());

    qDebug() << "ballast mass: " << ballastBody->GetMass();

    //ballast constraint, attach to monopile
    auto constraint_ballast = std::make_shared<ChLinkMateGeneric>();
    constraint_ballast->Initialize(ballastBody, monopile, ChFrame<>(ChCoordsys<>(ballastPos)));
    system.Add(constraint_ballast);

    //Init Load container
    auto loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    buoyancy = std::make_shared<Buoyancy>(p, loadcontainer, monopile);

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

    //complete setup
    system.Add(mesh);
    system.SetupInitial();

    //Initial rotation of the monopile
    //Rotate around x axis and y axis
    ChQuaternion<> qRotationX = Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_X);
    ChQuaternion<> qRotationZ= Q_from_AngAxis(20 * CH_C_DEG_TO_RAD, VECT_Z);
    //Translate to initial Position
    ChVector<> initPos = ChVector<>(0,0,0.5*p.towerHeight);

    //Define initial displacement
    ChCoordsys<> initCoords =ChCoordsys<>(initPos,qRotationX*qRotationZ);
    monopile->Move(initCoords);

    //system.DoStepDynamics(dT);

    //monopile->UpdateMarkers(dT);

    //Move mooring lines to the new position:
    for(auto & mooringLine : mooringLines) {
        mooringLine.updateFairleadNode();
    }


}

void PlatformModel::Reset(){

    system.Clear();
    system.SetChTime(0.0);
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
        //red/blue: ballast Node
        glColor4d(1,0,1,1);
        CVector ballastPos = CVecFromChVec(ballastNode->GetPos());
        glVertex3d(ballastPos.x, ballastPos.y, ballastPos.z);

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

    qDebug() << "updating";

    double old_step;
    double left_time;
    int restore_oldstep = FALSE;

    system.SetStep(dT);

    while (system.GetChTime() < endTime) {

        //qDebug() << "updating buoyancy again";
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
