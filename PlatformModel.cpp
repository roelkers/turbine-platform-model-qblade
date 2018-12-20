#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChQuaternion.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChMesh.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include "../XLLT/QLLTModule.h"
#include "../StructModel/StrModelDock.h"

#include <QtOpenGL>

#include "PlatformModel.h"
#include "PlatformParams.h"
#include "Monopile.h"

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




    int i=m_qlltSim->GetModule()->GetStrModelDock()->intBox->currentIndex();


    if (i==0)  system.SetTimestepperType(ChTimestepper::Type::HHT);
    else if (i==1)  system.SetTimestepperType(ChTimestepper::Type::NEWMARK);
    else if (i==2)  system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    else if (i==3)  system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    else if (i==4)  system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    else if (i==5)  system.SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL);
    else if (i==6)  system.SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED);
    else if (i==7)  system.SetTimestepperType(ChTimestepper::Type::LEAPFROG);
    else if (i==8)  system.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    else if (i==9)  system.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
    else if (i==10)  system.SetTimestepperType(ChTimestepper::Type::HEUN);


//    if (system.GetTimestepperType() == ChTimestepper::Type::NEWMARK){
//    auto mystepper = std::dynamic_pointer_cast<ChTimestepperNewmark>(system.GetTimestepper());
//    mystepper->SetGammaBeta(double(m_module->GetStrModelDock()->num_ex->value())/10.0,double(m_module->GetStrModelDock()->deg_ex->value())/10.0);
//    }

    if (system.GetTimestepperType() == ChTimestepper::Type::HHT){
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());

    mystepper->SetAlpha(-1.0/3.0);
    mystepper->SetStepControl(false);
    mystepper->SetMaxiters(int(m_qlltSim->GetModule()->GetStrModelDock()->iterBox->value()));
    mystepper->SetMode(ChTimestepperHHT::HHT_Mode::ACCELERATION);
    mystepper->SetModifiedNewton(true);
    mystepper->SetScaling(false);
    }

    // Change type of integrator:
//    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    //system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    //system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //system.SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL);
//    system.SetTimestepperType(ChTimestepper::Type::HHT);

//    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
//    mystepper->SetAlpha(-1.0/3.0);
//    mystepper->SetStepControl(false);
//    //    mystepper->SetMaxiters(int(qLLTSim->GetModule()->GetStrModelDock()->iterBox->value()));
//    mystepper->SetMaxiters(10);
//    mystepper->SetMode(ChTimestepperHHT::HHT_Mode::ACCELERATION);
//    mystepper->SetModifiedNewton(true);
//    mystepper->SetScaling(false);

    //Init Load container
    auto loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    //Calculate Position of Platform relative to tower base (interface)
    p.initPosInterface = ChVecFromCVec(m_qlltSim->initTrans);
    p.initRot = ChVecFromCVec(m_qlltSim->initRot);

    //moopile->getBody()->SetPos(initPosMonopile);

    //Create monopile
    monopile = std::make_shared<Monopile>(system, p, loadcontainer);

    //Define initial displacement
    //ChCoordsys<> initCoords =ChCoordsys<>(p.initPosVec,p.qRotationZ*p.qRotationY*p.qRotationX);
    //monopile->getBody()->Move(initCoords);

    qDebug() << "monopile pos x:" << monopile->getBody()->GetPos().x();
    qDebug() << "monopile pos y:" << monopile->getBody()->GetPos().y();
    qDebug() << "monopile pos z:" << monopile->getBody()->GetPos().z();

    ChQuaternion<> rotBody = monopile->getBody()->GetRot();

    qDebug() << "initial Rotation" << rotBody.Q_to_NasaAngles().x()/M_PI*180 << " " << rotBody.Q_to_NasaAngles().y()/M_PI*180 << " " << rotBody.Q_to_NasaAngles().z()/M_PI*180;

    //set initial velocity
    monopile->getBody()->SetPos_dt(p.initVelVec);

    //set angular velocity
    monopile->getBody()->SetWvel_par(p.initAngVelVec);

    //Add Gravity
    system.Set_G_acc(ChVector<>(0,0,-p.g));
    mesh->SetAutomaticGravity(true);
    //system.Set_G_acc(ChVector<>(0,0,0));

    //Add nacelle, tower and hub mass
    monopile->addMasses(system);

    //Angular increment of Mooring Line on Monopile
    double thetaInc;
    if(p.mooringLineNr>0){
        thetaInc = 360/p.mooringLineNr;
    }
    //Angle on Monopile of mooring line
    //start at 45 degrees to have symmetric roll and pitch forces for x and y
    double theta = 90;
    //Construct Mooring Lines
    for(int i = 0; i < p.mooringLineNr; i++){
        theta = theta + thetaInc;

        MooringLine mLine(system, mesh, p, theta, monopile,loadcontainer);
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

    //system.DoStepDynamics(m_qlltSim->getTimeStep());

    monopile->getBody()->SetBodyFixed(true);

//    for(int i = 0; i< p.nrRelaxations; i++){
//        system.DoStaticRelaxing(p.nrRelaxationSteps);
//    }

    monopile->getBody()->SetBodyFixed(false);

//    for(auto & mooringLine : mooringLines) {

//        //get reaction force from fairleads
//        ChVector<> reactForce = mooringLine.getConstraintMooring()->Get_react_force();
//        qDebug() << "reactForce mooring: " << reactForce.Length();

//        mooringLine.getTensionForce();
//    }

}

void PlatformModel::render(){

    glNewList(GLPLATFORM,GL_COMPILE);
    {
        monopile->render();

        for(auto & mooringLine : mooringLines) {
            mooringLine.render();
        }
    }
    glEndList();

}

void PlatformModel::update(double endTime, CVector aerolasticInterfaceForce, CVector aerolasticInterfaceTorque){

    double old_step;
    double left_time;
    int restore_oldstep = FALSE;

    double dT = m_qlltSim->GetStrTimestep();

    system.SetStep(dT);

    while (system.GetChTime() < endTime) {

        calculateSeaLevel(system.GetChTime());
        monopile->update(ChVecFromCVec(aerolasticInterfaceForce), ChVecFromCVec(aerolasticInterfaceTorque), seaLevel, system.GetChTime());

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

double PlatformModel::getTotalDragForce(){
    return monopile->getTotalDragForce();
}

double PlatformModel::getSeaLevel(){

    return seaLevel;
}

void PlatformModel::calculateSeaLevel(double time){

    seaLevel = p.waveAmplitude*cos(2*PI/p.wavePeriod*time);
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
    double angleNasa = rotBody.Q_to_NasaAngles().y()/M_PI*180;
    return angleNasa;
}

double PlatformModel::getPitchAngle(){

    ChQuaternion<> rotBody = monopile->getBody()->GetRot();

    //rotate monopile back from setup position to get correct angle
//    ChQuaternion<> qSetup = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);
//    ChQuaternion<> rotCorrected = rotBody*qSetup;
    double angleNasa = rotBody.Q_to_NasaAngles().x()/M_PI*180;
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

CVector PlatformModel::getInterfacePos(){
    return monopile->getInterfacePos();
}

ChQuaternion<> PlatformModel::getInterfaceRot(){
    return monopile->getInterfaceRot();
}
