#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChMesh.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include "PlatformModel.h"
#include "PlatformParams.h"
#include "Monopile.h"
#include "HydrodynamicDamping.h"

using namespace chrono;
using namespace chrono::fea;

PlatformModel::PlatformModel(QLLTSimulation *qLLTSim)
{

    m_qlltSim = qLLTSim;

    auto mkl_solver_speed = std::make_shared<ChSolverMKL<>>();

    system.SetSolver(mkl_solver_speed);

    mkl_solver_speed->ForceSparsityPatternUpdate(true);
    mkl_solver_speed->SetSparsityPatternLock(false);
    mkl_solver_speed->SetVerbose(false);

    //default settings of system
    system.SetTol(2e-4);
    system.SetTolForce(1e-3);
    system.SetSolverOverrelaxationParam(1);
    system.SetSolverSharpnessParam(1);

    // Change type of integrator:
    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    //Init Load container
    auto loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    //Create monopile
    monopile = std::make_shared<Monopile>(system, p, loadcontainer);

    //translate to rest position, here cylinder is static
//    double restPosition = calculateRestPositionOfPlatform();
//    qDebug() << "restPosition: " << restPosition;
//    ChVector<> restPosVec = ChVector<>(0,0,restPosition);

    //Define initial displacement
    ChCoordsys<> initCoords =ChCoordsys<>(p.initPosVec,p.qRotationZ*p.qRotationY*p.qRotationX);
    monopile->getBody()->Move(initCoords);

//    buoyancy = std::make_shared<Buoyancy>(p, loadcontainer, monopile);

//    hydrodynamicDamping = std::make_shared<HydrodynamicDamping>(p, loadcontainer, monopile);

    //Add Gravity
    system.Set_G_acc(ChVector<>(0,0,-p.g));

    //system.Set_G_acc(ChVector<>(0,0,0));

    //Angular increment of Mooring Line on Monopile
    double thetaInc;
    if(p.mooringLineNr>0){
        thetaInc = 360/p.mooringLineNr;
    }
    //Angle on Monopile of mooring line
    //start at 45 degrees to have symmetric roll and pitch forces for x and y
    double theta = 0;
    //Construct Mooring Lines
    for(int i = 0; i < p.mooringLineNr; i++){
        theta = theta + thetaInc;

        MooringLine mLine(system, mesh, p, theta, monopile);
        mooringLines.push_back(mLine);
    }

    //complete setup
    system.Add(mesh);
    system.SetupInitial();
    system.Update();
    system.Setup();

    //Set Rest position and rest length of mooring lines
    for(auto & mooringLine : mooringLines) {
        mooringLine.setRestLengthAndPosition();

    }

    system.DoStepDynamics(m_qlltSim->getTimeStep());

    for(auto & mooringLine : mooringLines) {

        //get reaction force from fairleads
        ChVector<> reactForce = mooringLine.getConstraintMooring()->Get_react_force();
        qDebug() << "reactForce mooring: " << reactForce.Length();

        mooringLine.getTensionForce();
    }

}

//double PlatformModel::calculateRestPositionOfPlatform(){
//    //To set the monopile at the rest position the gravity force and buoyancy force have to be equal at that point
//    double massTotal = monopile->getBody()->GetMass()+p.nacelleMass+p.ballastMass;
//    double areaMonopile = pow(p.towerRadius,2)*M_PI;

//    //we get the length by evaluating the force balance
//    double displacedLength = massTotal/(areaMonopile*p.rhoWater);
//    qDebug() << "displaced Length: " << displacedLength;

//    double force_gravity = massTotal*p.g;
//    double force_buoyancy = displacedLength*areaMonopile*p.rhoWater*p.g;

//    qDebug() << "force_gravity: " << force_gravity;
//    qDebug() << "force_buoyancy: " << force_buoyancy;

//    return p.seaLevel+0.5*p.towerHeight-displacedLength;
//}

void PlatformModel::render(){

    glNewList(GLPLATFORM,GL_COMPILE);
    {
        monopile->render();
//        buoyancy->render();
//        hydrodynamicDamping->render();

        for(auto & mooringLine : mooringLines) {
            mooringLine.render();
        }
    }
    glEndList();

}

void PlatformModel::update(double endTime){

    double old_step;
    double left_time;
    int restore_oldstep = FALSE;

    double dT = m_qlltSim->getTimeStep();

    system.SetStep(dT);

    while (system.GetChTime() < endTime) {

        //qDebug() << "updating buoyancy again";
        monopile->update();
//        buoyancy->update();
//        hydrodynamicDamping->update();

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

double PlatformModel::getXPosition(){

    return monopile->getBody()->GetPos().x();

}

double PlatformModel::getYPosition(){

    return monopile->getBody()->GetPos().y();
}

double PlatformModel::getZPosition(){

    return monopile->getBody()->GetPos().z();
}

double PlatformModel::getRollAngle(){

    ChQuaternion<> rotBody = monopile->getBody()->GetRot();

    //rotate monopile back from setup position to get correct angle
//    ChQuaternion<> qSetup = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);
//    ChQuaternion<> rotCorrected = rotBody*qSetup;
    double angleNasa = rotBody.Q_to_NasaAngles().x()/M_PI*180;
    return angleNasa;
}

double PlatformModel::getPitchAngle(){

    ChQuaternion<> rotBody = monopile->getBody()->GetRot();

    //rotate monopile back from setup position to get correct angle
//    ChQuaternion<> qSetup = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);
//    ChQuaternion<> rotCorrected = rotBody*qSetup;
    double angleNasa = rotBody.Q_to_NasaAngles().y()/M_PI*180;
    return angleNasa;
}

double PlatformModel::getYawAngle(){

    ChQuaternion<> rotBody = monopile->getBody()->GetRot();

      //rotate monopile back from setup position to get correct angle
//    ChQuaternion<> qSetup = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);
//    ChQuaternion<> rotCorrected = rotBody*qSetup;
    double angleNasa = rotBody.Q_to_NasaAngles().z()/M_PI*180;
    return angleNasa;
}

//double PlatformModel::getDragForceZBottom(){

//    ChVector<> force = hydrodynamicDamping->getDragForceZBottom()->GetForce();
//    return force.Length();
//}

