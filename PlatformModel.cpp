#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_fea/ChLinkPointFrame.h"
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
    //Reset();

    qDebug() << "tower height: " << p.towerHeight;

    // Change solver settings
    /*
    system.SetSolverType(ChSolver::Type::MINRES);
    system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    system.SetMaxItersSolverSpeed(200);
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto solver = std::static_pointer_cast<ChSolverMINRES>(system.GetSolver());
    solver->SetVerbose(false);
    solver->SetDiagonalPreconditioning(true);
    */
    auto mkl_solver_speed = std::make_shared<ChSolverMKL<>>();

    system.SetSolver(mkl_solver_speed);

    mkl_solver_speed->ForceSparsityPatternUpdate(true);
    mkl_solver_speed->SetSparsityPatternLock(false);
    mkl_solver_speed->SetVerbose(false);

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

    //Initial rotation of the monopile
    //Rotate around x axis and y axis
    ChQuaternion<> qRotationX = Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_X);
    ChQuaternion<> qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z);
    //Translate to initial Position
    double restPosition = calculateRestPositionOfPlatform();
    qDebug() << "restPosition: " << restPosition;
    ChVector<> initPos = ChVector<>(0,0,restPosition);

    //Define initial displacement
    ChCoordsys<> initCoords =ChCoordsys<>(initPos,qRotationX*qRotationZ);
    monopile->Move(initCoords);

    //Create ballast on the bottom of the monopile

    ballastBody = std::make_shared<ChBody>();
    ballastBody->SetMass(p.ballastMass);
    system.Add(ballastBody);
    //Move to position in local frame, on the bottom end
    ChVector<> ballastPos = monopile->TransformPointLocalToParent(ChVector<>(0,-0.5*p.towerHeight,0)); //local frame to transform
    std::shared_ptr<ChNodeFEAxyzD> ballastNode = std::make_shared<ChNodeFEAxyzD>(ballastPos, p.towerSetupDir);
    mesh->AddNode(ballastNode);
    ballastBody->SetPos(ballastNode->GetPos());
    ballastBody->SetRot(qSetup);
    qDebug() << "ballast mass: " << ballastBody->GetMass();
    //ballast constraint, attach to monopile
    std::shared_ptr<ChLinkMateFix> constraint_ballast = std::make_shared<ChLinkMateFix>();
    constraint_ballast->Initialize(ballastBody, monopile);
    system.Add(constraint_ballast);

    //Create nacelle mass on the top of the monopile

    nacelleBody = std::make_shared<ChBody>();
    nacelleBody->SetMass(p.nacelleMass);
    system.Add(nacelleBody);
    //Move to position in local frame, on the top end
    ChVector<> nacellePos = monopile->TransformPointLocalToParent(ChVector<>(0,0.5*p.towerHeight,0)); //local frame to transform
    std::shared_ptr<ChNodeFEAxyzD> nacelleNode = std::make_shared<ChNodeFEAxyzD>(nacellePos, p.towerSetupDir);
    mesh->AddNode(nacelleNode);
    nacelleBody->SetPos(nacelleNode->GetPos());
    nacelleBody->SetRot(qSetup);
    qDebug() << "nacelle mass: " << nacelleBody->GetMass();
    //nacelle constraint, attach to monopile
    std::shared_ptr<ChLinkMateFix> constraint_nacelle = std::make_shared<ChLinkMateFix>();
    constraint_nacelle->Initialize(nacelleBody, monopile);
    system.Add(constraint_nacelle);

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
        //MooringLine mLine(system, mesh, p, theta, monopile);
        //mooringLines.push_back(mLine);

    }

    //complete setup
    system.Add(mesh);
    system.SetupInitial();
    system.Update();
    system.Setup();

    //Set Rest position and rest length of mooring lines

    for(auto & mooringLine : mooringLines) {
        //mooringLine.setRestLengthAndPosition();
    }

}

double PlatformModel::calculateRestPositionOfPlatform(){
    //To set the monopile at the rest position the gravity force and buoyancy force have to be equal at that point
    double massTotal = monopile->GetMass()+p.nacelleMass+p.ballastMass;
    double areaMonopile = pow(p.towerRadius,2)*M_PI;

    //we get the length by evaluating the force balance
    double displacedLength = massTotal/(areaMonopile*p.rhoWater);

    double force_gravity = massTotal*p.g;
    double force_buoyancy = displacedLength*areaMonopile*p.rhoWater*p.g;

    qDebug() << "force_gravity: " << force_gravity;
    qDebug() << "force_buoyancy: " << force_buoyancy;

    return p.seaLevel+0.5*p.towerHeight-displacedLength;
}

void PlatformModel::render(){

    glNewList(GLPLATFORM,GL_COMPILE);
    {
        renderMonopile();
        //renderMooringLines();
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
        //red/blue: ballast Body
        glColor4d(1,0,1,1);
        CVector ballastPos = CVecFromChVec(ballastBody->GetPos());
        glVertex3d(ballastPos.x, ballastPos.y, ballastPos.z);
        //light red/green: nacelle Body
        glColor4d(0.5,1,0,1);
        CVector nacellePos = CVecFromChVec(nacelleBody->GetPos());
        glVertex3d(nacellePos.x, nacellePos.y, nacellePos.z);

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
