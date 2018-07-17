#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLinkMate.h"

#include "../GlobalFunctions.h"
#include "../XLLT/QLLTSimulation.h"
#include <QtOpenGL>

#include <QDebug>

#include "PlatformParams.h"
#include "Monopile.h"
#include "MonopileElement.h"

using namespace chrono;
using namespace chrono::fea;
std::shared_ptr<chrono::ChLoadBodyForce> Monopile::getAddedDampingForce() const
{
    return addedDampingForce;
}

Monopile::Monopile(ChSystem &system, PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer)
    :p(p)
{

    platformBody= std::make_shared<ChBody>();
    //set cylinder to setup position
    platformBody->SetPos(p.platformSetupPos);

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

    ChVector<> zStart = p.platformSetupPos + vecGtoE;

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

    for(int i = 0; i<p.platformNrElementsBelowTaper; i++){
        B = A+platformLengthOfElementBelowTaper*zAxisMonopile;

        MonopileElement monopileElement(p,loadContainer,platformBody,platformLengthOfElementBelowTaper,A,B,crossSectionAreaBelowTaper,volume);
        monopileElements.push_back(monopileElement);

        A = B;
    }

    //Taper elements
    double taperLengthOfElement = p.platformLengthTaper/p.platformNrElementsTaper;
    double crossSectionAreaTaperElement = 0;
    double currentTaperRadius = 0;
    double zTaper = 0.5*taperLengthOfElement;

    qDebug()<< "lengthOfElement taper: " << taperLengthOfElement;

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

        A = B;
        zTaper = zTaper + taperLengthOfElement;
    }

    //Create elements Above taper
    double platformLengthOfElementAboveTaper = p.platformLengthAboveTaper/p.platformNrElementsAboveTaper;
    double crossSectionAreaAboveTaper = 2*p.platformRadiusAboveTaper*platformLengthOfElementAboveTaper;
    volume = M_PI*pow(p.platformRadiusAboveTaper,2)*platformLengthOfElementAboveTaper;

    qDebug()<< "element volume Above taper: " << volume;
    qDebug()<< "lengthOfElement Above taper: " << platformLengthOfElementAboveTaper;
    qDebug()<< "crossSectionAreaAboveTaper: " << crossSectionAreaAboveTaper;

    for(int i = 0; i<p.platformNrElementsAboveTaper; i++){
        B = A+platformLengthOfElementAboveTaper*zAxisMonopile;

        MonopileElement monopileElement(p,loadContainer,platformBody,platformLengthOfElementAboveTaper,A,B,crossSectionAreaAboveTaper,volume);
        monopileElements.push_back(monopileElement);

        A = B;
    }

    //Create tower elements
    double towerLengthOfElement = p.towerHeight/p.nrElementsTower;
    double crossSectionAreaTowerElement = 0;
    double currentTowerRadius = 0;
    double z_tower = 0.5*towerLengthOfElement;

    for(int i = 0; i<p.nrElementsTower; i++){

        B = A+towerLengthOfElement*zAxisMonopile;
        //calculate linear tower radius
        currentTowerRadius = (p.towerRadiusTop-p.platformRadiusAboveTaper)/p.towerHeight*z_tower + p.platformRadiusAboveTaper;

        volume = M_PI*pow(currentTowerRadius,2)*towerLengthOfElement;
        crossSectionAreaTowerElement = 2*currentTowerRadius*towerLengthOfElement;

        MonopileElement monopileElement(p,loadContainer,platformBody,towerLengthOfElement,A,B,crossSectionAreaTowerElement ,volume);
        monopileElements.push_back(monopileElement);

        qDebug()<< "element volume tower: " << volume;
        qDebug()<< "z tower: " << z_tower;
        qDebug()<< "crossSectionArea tower:  " << crossSectionAreaTowerElement;

        A = B;
        z_tower = z_tower + towerLengthOfElement;
    }

    addedDampingForce = std::make_shared<ChLoadBodyForce> (
      platformBody, //platformBody
      ChVector<>(0,0,0), //initialize
      true, //local force
      platformBody->GetPos(), //bottom marker position is attack point of force
      true //local point
    );

    addedYawDampingTorque = std::make_shared<ChLoadBodyTorque>(
      platformBody,
      ChVector<>(0,0,0), //initialize
      true //local torque
    );

    loadContainer->Add(addedYawDampingTorque);
    loadContainer->Add(addedDampingForce);
}


void Monopile::addMasses(ChSystem& system){

    ChVector<> vecGtoI = ChVector<>(0,0,-p.distanceGtoE+p.platformLengthBelowTaper+p.platformLengthTaper+p.platformLengthAboveTaper);
    ChVector<> vecItoY = ChVector<>(0,0,p.towerHeight);
    ChVector<> vecYtoN = ChVector<>(-p.nacelleDistanceDownstream,0,p.nacelleDistanceToYawBearing);
    ChVector<> vecYtoH = ChVector<>(p.hubDistanceUpstream,0,p.hubDistanceToYawBearing);
    ChVector<> vecItoT = ChVector<>(0,0,p.towerCOGDistanceFromBottom);

    ChVector<> nacellePos = platformBody->TransformPointLocalToParent(vecGtoI+vecItoY+vecYtoN);
    nacelleBody = std::make_shared<ChBody>();
    nacelleBody->SetPos(nacellePos);
    nacelleBody->SetMass(p.nacelleMass);
    system.Add(nacelleBody);

    qDebug() << "nacelle mass:" << nacelleBody->GetMass();

    std::shared_ptr<ChLinkMateFix> nacelleConstraint = std::make_shared<ChLinkMateFix>();
    nacelleConstraint->Initialize(platformBody,nacelleBody);
    system.Add(nacelleConstraint);

    ChVector<> hubPos = platformBody->TransformPointLocalToParent(vecGtoI+vecItoY+vecYtoH);
    hubBody = std::make_shared<ChBody>();
    hubBody->SetPos(hubPos);
    hubBody->SetMass(p.hubMass);
    hubBody->SetRot(platformBody->GetRot());
    system.Add(hubBody);

    //qDebug() << "nacelle mass:" << nacelleBody->GetMass();

    std::shared_ptr<ChLinkMateFix> hubConstraint = std::make_shared<ChLinkMateFix>();
    hubConstraint->Initialize(platformBody,hubBody);
    system.Add(hubConstraint);

    ChVector<> towerPos = platformBody->TransformPointLocalToParent(vecGtoI+vecItoT);
    towerBody = std::make_shared<ChBody>();
    towerBody->SetPos(towerPos);
    towerBody->SetMass(p.towerMass);
    system.Add(towerBody);

    std::shared_ptr<ChLinkMateFix> towerConstraint = std::make_shared<ChLinkMateFix>();
    towerConstraint->Initialize(platformBody,towerBody);
    system.Add(towerConstraint);

    //Create Blades
    double thetaInc;
    if(p.bladeNr>0){
        thetaInc = 360/p.bladeNr;
    }
    double theta = 0;

    std::shared_ptr<ChBody> bladeBody;
    ChVector<> bladePos;
    double zBlade = 0;
    double yBlade = 0;
    double totalBladeMass = 0;
    for(int i = 0; i < p.bladeNr; i++){
        theta = theta + thetaInc;
        bladeBody = std::make_shared<ChBody>();
        bladeBody->SetMass(p.bladeMass);

        double xStart = p.mooringRadiusToFairleadsFromCenter*sin(theta/180*M_PI);
        double yStart = p.mooringRadiusToFairleadsFromCenter*cos(theta/180*M_PI);

        double bladeCOGRadiusFromHubAxis = (p.bladeCOGDistanceFromRoot+p.hubDiameter/2);

        zBlade = bladeCOGRadiusFromHubAxis*sin(theta/180*M_PI);
        yBlade = bladeCOGRadiusFromHubAxis*cos(theta/180*M_PI);

        bladePos = hubBody->TransformPointLocalToParent(ChVector<>(0,yBlade,zBlade));

        bladeBody->SetPos(bladePos);
        bladeBodies.push_back(bladeBody);

        system.Add(bladeBody);

        std::shared_ptr<ChLinkMateFix> bladeConstraint = std::make_shared<ChLinkMateFix>();
        bladeConstraint->Initialize(hubBody,bladeBody);
        system.Add(bladeConstraint);

        qDebug()<< "creating blade nr:" << i;
        qDebug()<< "theta blade: " << theta;
        qDebug()<< "yBlade: " << yBlade;
        qDebug()<< "zBlade: " << zBlade;

        totalBladeMass += bladeBody->GetMass();
    }

    double totalMass = platformBody->GetMass() + hubBody->GetMass() + towerBody->GetMass() + nacelleBody->GetMass() + totalBladeMass;
    qDebug() << "Total mass of all bodies: " << totalMass;
}

void Monopile::update(){

    //update elements
    for(auto &element : monopileElements){
        element.update();
    }


    //qDebug() << "markerVelocity z" << markerVelZ;

    double addedDampingForceX = -platformBody->GetPos_dt().x()*p.addedDampingX;
    double addedDampingForceY = -platformBody->GetPos_dt().y()*p.addedDampingY;
    double addedDampingForceZ = -platformBody->GetPos_dt().z()*p.addedDampingZ;

    ChVector<> addedDampingForceVec = ChVector<>(addedDampingForceX,addedDampingForceY,addedDampingForceZ);

    addedDampingForce->SetForce(addedDampingForceVec,true);

    //Calculate additional damping of platform around yaw-axis

    //Angular speed expressed in local coords
    ChVector<> platformAngularSpeed = platformBody->GetWvel_loc();

    double platformYawSpeed = platformAngularSpeed.z();
    ChVector<> torque = ChVector<>(0,0,-platformYawSpeed*p.addedDampingYaw);

    //qDebug() << "yaw damping" << -platformYawSpeed*p.addedDampingYaw;

    addedYawDampingTorque->SetTorque(torque);
}

void Monopile::render(){
    glPointSize(10);
    glLineWidth(3);

    //Draw viz vertices
    glBegin(GL_POINTS);
        //blue: body
        glColor4d(0,0,1,1);
        CVector monopilePos = CVecFromChVec(platformBody->GetPos());
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        //strong red, green: nacelle //sky blue
        glColor4d(0.75,1,0,1);
        CVector nacellePos = CVecFromChVec(nacelleBody->GetPos());
        glVertex3d(nacellePos.x,nacellePos.y,nacellePos.z);
        //strong green, blue: hub //olive
        glColor4d(0,0.75,1,1);
        CVector hubPos = CVecFromChVec(hubBody->GetPos());
        glVertex3d(hubPos.x,hubPos.y,hubPos.z);

        //strong red, blue, green: tower //dark pink
        glColor4d(0.75,1,1,1);
        CVector towerPos = CVecFromChVec(towerBody->GetPos());
        glVertex3d(towerPos.x,towerPos.y,towerPos.z);
        //strong blue, red: blades
        glColor4d(1,0,0.75,1);
        for(auto &blade : bladeBodies){
            CVector bladePos = CVecFromChVec(blade->GetPos());
            glVertex3d(bladePos.x,bladePos.y,bladePos.z);
        }
    glEnd();

    glPointSize(0.1);
    glBegin(GL_LINES);

        //Draw Coordinate system of monopile
        ChCoordsys<> monopileCoord = platformBody->GetCoord();
        //red: x-axis
        glColor4d(1,0,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector xAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetXaxis());
        glVertex3d(xAxisEnd.x,xAxisEnd.y,xAxisEnd.z);
        //green: y-axis
        glColor4d(0,1,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector yAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetYaxis());
        glVertex3d(yAxisEnd.x,yAxisEnd.y,yAxisEnd.z);
        //blue: z-axis
        glColor4d(0,0,1,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        CVector zAxisEnd = CVecFromChVec(platformBody->GetPos()+p.cSystemFactor*monopileCoord.rot.GetZaxis());
        glVertex3d(zAxisEnd.x,zAxisEnd.y,zAxisEnd.z);

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
        glColor4d(1,0.5,0,1);
        glVertex3d(monopilePos.x,monopilePos.y,monopilePos.z);
        ChVector<> addedDampingForceAbs = addedDampingForce->GetForce();
        CVector bottomForceEnd = CVecFromChVec(platformBody->GetPos()+addedDampingForceAbs*p.forceLineFactor);
        glVertex3d(bottomForceEnd.x,bottomForceEnd.y,bottomForceEnd.z);
    glEnd();
    //Render Elements
    for(auto &element : monopileElements){
        element.render();
    }
}

