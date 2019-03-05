#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/core/ChQuaternion.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "PlatformParams.h"
#include "Monopile.h"
#include "MonopileElement.h"

using namespace chrono;
using namespace chrono::fea;

Monopile::Monopile(ChSystem &system, PlatformParams &p, std::shared_ptr<ChLoadContainer> loadContainer)
    :p(p)
{

    //interface force & torque
    interfaceBody = std::make_shared<ChBody>();
    system.Add(interfaceBody);

    p.initRotQuat.Q_from_NasaAngles(ChVector<>(p.initRot.y()/180*PI,p.initRot.x()/180*PI,p.initRot.z()/180*PI));

    interfaceBody->SetPos(p.initPosInterface);
    interfaceBody->SetRot(p.initRotQuat);

    platformBody= std::make_shared<ChBody>();

    ChVector<> vecGtoI = ChVector<>(0,0,-p.distanceGtoE+p.platformLengthBelowTaper+p.platformLengthTaper+p.platformLengthAboveTaper);
    p.initPosVec = interfaceBody->TransformPointLocalToParent(-vecGtoI);

    platformBody->SetPos(p.initPosVec);
    platformBody->SetRot(p.initRotQuat);

    std::shared_ptr<ChLinkMateFix> interfaceConstraint = std::make_shared<ChLinkMateFix>();
    interfaceConstraint->Initialize(platformBody,interfaceBody);
    system.Add(interfaceConstraint);

    ChVector<> inertiaVector = ChVector<>(p.platformMassMomentInertiaInRollAndPitch,p.platformMassMomentInertiaInRollAndPitch,p.platformMassMomentInertiaInYaw);

    platformBody->SetMass(p.platformMass);
    platformBody->SetInertiaXX(inertiaVector);

    system.Add(platformBody);

    qDebug() << "platformBody mass: " << platformBody->GetMass();
    qDebug() << "platformBody inertia xx" << platformBody->GetInertiaXX().x();
    qDebug() << "platformBody inertia yy" << platformBody->GetInertiaXX().y();
    qDebug() << "platformBody inertia zz" << platformBody->GetInertiaXX().z();

    ChVector<> zAxisMonopile = platformBody->GetFrame_COG_to_abs().GetRot().GetZaxis();

    ChVector<> vecGtoE = ChVector<>(0,0,-p.distanceGtoE);

    ChVector<> zStart = platformBody->TransformPointLocalToParent(vecGtoE);

    //Create elements below taper
    double platformLengthOfElementBelowTaper = p.platformLengthBelowTaper/p.platformNrElementsBelowTaper;
    double crossSectionAreaBelowTaper = 2*p.platformRadiusBelowTaper*platformLengthOfElementBelowTaper;
    double volume = M_PI*pow(p.platformRadiusBelowTaper,2)*platformLengthOfElementBelowTaper;

    qDebug()<< "element volume below taper: " << volume;
    qDebug()<< "lengthOfElement below taper: " << platformLengthOfElementBelowTaper;
    qDebug()<< "crossSectionAreaBelowTaper: " << crossSectionAreaBelowTaper;

    ChVector<> A;
    ChVector<> B;
    A = zStart;
    double totalVolumeBelowTaper = 0;

    for(int i = 0; i<p.platformNrElementsBelowTaper; i++){
        B = A+platformLengthOfElementBelowTaper*zAxisMonopile;

        MonopileElement monopileElement(p,loadContainer,platformBody,platformLengthOfElementBelowTaper,A,B,crossSectionAreaBelowTaper,volume);
        monopileElements.push_back(monopileElement);

        totalVolumeBelowTaper = totalVolumeBelowTaper + volume;
        A = B;
    }

    qDebug() << "totalVolume below Taper of Platform" << totalVolumeBelowTaper;

    //Taper elements
    double taperLengthOfElement = p.platformLengthTaper/p.platformNrElementsTaper;
    double crossSectionAreaTaperElement = 0;
    double currentTaperRadius = 0;
    double zTaper = 0.5*taperLengthOfElement;

    qDebug()<< "lengthOfElement taper: " << taperLengthOfElement;

    double totalVolumeTaper = 0;

    for(int i = 0; i<p.platformNrElementsTaper; i++){

        B = A+taperLengthOfElement*zAxisMonopile;
        currentTaperRadius = (p.platformRadiusAboveTaper-p.platformRadiusBelowTaper)/p.platformLengthTaper*zTaper + p.platformRadiusBelowTaper;

        volume = M_PI*pow(currentTaperRadius,2)*taperLengthOfElement;
        crossSectionAreaTaperElement = 2*currentTaperRadius*taperLengthOfElement;

        MonopileElement monopileElement(p,loadContainer,platformBody,taperLengthOfElement,A,B,crossSectionAreaTaperElement ,volume);
        monopileElements.push_back(monopileElement);

        qDebug()<< "current taper radius" << currentTaperRadius;
        qDebug()<< "element volume taper: " << volume;
        qDebug()<< "z taper: " << zTaper;
        qDebug()<< "crossSectionArea taper:  " << crossSectionAreaTaperElement;

        totalVolumeTaper = totalVolumeTaper + volume;

        A = B;
        zTaper = zTaper + taperLengthOfElement;
    }

    qDebug() << "totalVolume Taper of Platform" << totalVolumeTaper;

    //Create elements Above taper
    double platformLengthOfElementAboveTaper = p.platformLengthAboveTaper/p.platformNrElementsAboveTaper;
    double crossSectionAreaAboveTaper = 2*p.platformRadiusAboveTaper*platformLengthOfElementAboveTaper;
    volume = M_PI*pow(p.platformRadiusAboveTaper,2)*platformLengthOfElementAboveTaper;

    qDebug()<< "element volume Above taper: " << volume;
    qDebug()<< "lengthOfElement Above taper: " << platformLengthOfElementAboveTaper;
    qDebug()<< "crossSectionAreaAboveTaper: " << crossSectionAreaAboveTaper;

    double totalVolumeAboveTaper = 0;

    for(int i = 0; i<p.platformNrElementsAboveTaper; i++){
        B = A+platformLengthOfElementAboveTaper*zAxisMonopile;

        MonopileElement monopileElement(p,loadContainer,platformBody,platformLengthOfElementAboveTaper,A,B,crossSectionAreaAboveTaper,volume);
        monopileElements.push_back(monopileElement);

        totalVolumeAboveTaper = totalVolumeAboveTaper +volume;

        A = B;
    }

    qDebug() << "totalVolume Above Taper of Platform" << totalVolumeAboveTaper;

    addedDampingForce = std::make_shared<ChLoadBodyForce> (
      platformBody, //platformBody
      ChVector<>(0,0,0), //initialize
      true, //local force
      platformBody->GetPos(),
      true //local point
    );

    addedYawSpringTorque = std::make_shared<ChLoadBodyTorque>(
      platformBody,
      ChVector<>(0,0,0), //initialize
      true //local torque
    );

    addedYawDampingTorque = std::make_shared<ChLoadBodyTorque>(
      platformBody,
      ChVector<>(0,0,0), //initialize
      true //local torque
    );

    aerolasticInterfaceForce = std::make_shared<ChLoadBodyForce> (
      platformBody, //platformBody
      ChVector<>(0,0,0), //initialize
      false, //local force
      platformBody->TransformPointLocalToParent(vecGtoI),
      //platformBody->GetPos(),
      false //local point
    );

    aerolasticInterfaceTorque = std::make_shared<ChLoadBodyTorque>(
      platformBody,
      ChVector<>(0,0,0), //initialize
      false //local torque
    );

    loadContainer->Add(addedYawDampingTorque);
    loadContainer->Add(addedYawSpringTorque);
    loadContainer->Add(addedDampingForce);

    loadContainer->Add(aerolasticInterfaceForce);
    loadContainer->Add(aerolasticInterfaceTorque);

}

void Monopile::update(ChVector<> const& interfaceForceVec, ChVector<> const& interfaceTorqueVec, double seaLevel, double time){

    //update elements
    for(auto &element : monopileElements){
       element.update(seaLevel,time);
    }

    double addedDampingForceX = -platformBody->GetPos_dt().x()*p.addedDampingX;
    double addedDampingForceY = -platformBody->GetPos_dt().y()*p.addedDampingY;
    double addedDampingForceZ = -platformBody->GetPos_dt().z()*p.addedDampingZ;

    ChVector<> addedDampingForceVec = ChVector<>(addedDampingForceX,addedDampingForceY,addedDampingForceZ);
    addedDampingForce->SetForce(addedDampingForceVec,true);

    //Calculate torque due to artifical yaw spring stiffness
    double addedYawSpringTorqueZ = -platformBody->GetRot().Q_to_NasaAngles().z()*p.addedYawSpringStiffness;

    ChVector<> yawSpringTorque = ChVector<>(0,0,addedYawSpringTorqueZ);
    addedYawSpringTorque->SetTorque(addedYawSpringTorqueZ);

    //Calculate additional damping of platform around yaw-axis

    //Angular speed expressed in local coords
    ChVector<> platformAngularSpeed = platformBody->GetWvel_loc();
    double platformYawSpeed = platformAngularSpeed.z();
    ChVector<> torque = ChVector<>(0,0,-platformYawSpeed*p.addedDampingYaw);
    addedYawDampingTorque->SetTorque(torque);

    //update forces & torques at interface
    ChVector<> vecGtoI = ChVector<>(0,0,-p.distanceGtoE+p.platformLengthBelowTaper+p.platformLengthTaper+p.platformLengthAboveTaper);

    aerolasticInterfaceForce->SetApplicationPoint(platformBody->TransformPointLocalToParent(vecGtoI),false);
    interfaceBody->SetPos(platformBody->TransformPointLocalToParent(vecGtoI));

    aerolasticInterfaceForce->SetForce(interfaceForceVec, false);
    aerolasticInterfaceTorque->SetTorque(interfaceTorqueVec);
}

void Monopile::render() const{
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //blue: body
        glColor4d(0,0,1,1);
        CVector monopilePos = CVecFromChVec(platformBody->GetPos());
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
//        //strong red, green: nacelle //sky blue
//        glColor4d(0.75,1,0,1);
//        CVector nacellePos = CVecFromChVec(nacelleBody->GetPos());
//        glVertex3d(nacellePos.x,nacellePos.y,nacellePos.z);
//        //strong green, blue: hub //olive
//        glColor4d(0,0.75,1,1);
//        CVector hubPos = CVecFromChVec(hubBody->GetPos());
//        glVertex3d(hubPos.x,hubPos.y,hubPos.z);

//        //strong red, blue, green: tower //dark pink
//        glColor4d(0.75,1,1,1);
//        CVector towerPos = CVecFromChVec(towerBody->GetPos());
//        glVertex3d(towerPos.x,towerPos.y,towerPos.z);
//        //strong blue, red: blades
//        glColor4d(1,0,0.75,1);
//        for(auto &blade : bladeBodies){
//            CVector bladePos = CVecFromChVec(blade->GetPos());
//            glVertex3d(bladePos.x,bladePos.y,bladePos.z);
//        }
        //Draw Interface Body: blue
        CVector interfacePos = CVecFromChVec(interfaceBody->GetPos());
        glPointSize(20);
        glColor4d(0,1,0,1);
        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
    glEnd();

    glPointSize(0.1);
    glBegin(GL_LINES);

        //Draw Coordinate system of monopile
//        ChCoordsys<> monopileCoord = platformBody->GetCoord();
//        //red: x-axis
//        glColor4d(1,0,0,1);
//        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
//        CVector xAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetXaxis());
//        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
//        //green: y-axis
//        glColor4d(0,1,0,1);
//        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
//        CVector yAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetYaxis());
//        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
//        //blue: z-axis
//        glColor4d(0,0,1,1);
//        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
//        CVector zAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetZaxis());
//        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);

//        //Draw Coordinate system of interface
//        ChCoordsys<> interfaceCoord = interfaceBody->GetCoord();
//        //red: x-axis
//        glColor4d(1,0,0,1);
//        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
//        xAxisEnd = CVecFromChVec(interfaceBody->GetPos()+p.cSystemFactor*interfaceCoord.rot.GetXaxis());
//        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
//        //green: y-axis
//        glColor4d(0,1,0,1);
//        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
//        yAxisEnd = CVecFromChVec(interfaceBody->GetPos()+p.cSystemFactor*interfaceCoord.rot.GetYaxis());
//        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
//        //blue: z-axis
//        glColor4d(0,0,1,1);
//        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
//        zAxisEnd = CVecFromChVec(interfaceBody->GetPos()+p.cSystemFactor*interfaceCoord.rot.GetZaxis());
//        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);

        //Draw Interface Force: green
//        glColor4d(0.5,1,0,1);
//        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
//        ChVector<> aerolasticInterfaceForceAbs = aerolasticInterfaceForce->GetForce();
//        CVector aerolasticForceEnd = CVecFromChVec(interfaceBody->GetPos()+aerolasticInterfaceForceAbs*p.forceLineFactor);
//        glVertex3d(aerolasticForceEnd.x,aerolasticForceEnd.y,aerolasticForceEnd.z);

        //Draw Interface Torque : red
//        glColor4d(1,0,0.5,1);
//        glVertex3d(interfacePos.x,interfacePos.y,interfacePos.z);
//        ChVector<> aerolasticInterfaceTorqueAbs = aerolasticInterfaceTorque->GetTorque();
//        CVector aerolasticTorqueEnd = CVecFromChVec(interfaceBody->GetPos()+aerolasticInterfaceTorqueAbs*p.forceLineFactor);
//        glVertex3d(aerolasticTorqueEnd.x,aerolasticTorqueEnd.y,aerolasticTorqueEnd.z);
//        //hub c-system
//        //red: x-axis
//        ChCoordsys<> hubCoord = hubBody->GetCoord();
//        glColor4d(1,0,0,1);
//        glVertex3d(hubPos.x,hubPos.y,hubPos.z);
//        xAxisEnd = CVecFromChVec(hubBody->GetPos()+p.cSystemFactor*hubCoord.rot.GetXaxis());
//        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
//        //green: y-axis
//        glColor4d(0,1,0,1);
//        glVertex3d(hubPos.x,hubPos.y,hubPos.z);
//        yAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*hubCoord.rot.GetYaxis());
//        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
//        //blue: z-axis
//        glColor4d(0,0,1,1);
//        glVertex3d(hubPos.x,hubPos.y,hubPos.z);
//        zAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*hubCoord.rot.GetZaxis());
//        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);

        //Draw Added Damping Force: light green
//        glColor4d(1,0.5,0,1);
//        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
//        ChVector<> addedDampingForceAbs = addedDampingForce->GetForce();
//        CVector bottomForceEnd = CVecFromChVec(platformBody->GetPos()+addedDampingForceAbs*p.forceLineFactor);
//        glVertex3d(bottomForceEnd.x,bottomForceEnd.y,bottomForceEnd.z);
    glEnd();
    //Render Elements
    for(auto &element : monopileElements){
        element.render();
    }
}

CVector Monopile::getInterfacePos() const{
    return CVecFromChVec(interfaceBody->GetPos());
}

chrono::ChQuaternion<> Monopile::getInterfaceRot() const{
    return interfaceBody->GetRot();
}
