
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
Monopile::Monopile(ChSystem &system, PlatformParams p, std::shared_ptr<ChLoadContainer> loadContainer)
    :p(p)
{

    qDebug() << "creating cylinder";
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

    ChVector<> vecGtoE = ChVector<>(0,0,p.distanceGtoE);

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

    //Taper element
    B = A+p.platformLengthTaper*zAxisMonopile;
    double crossSectionAreaTaper = 2*(p.platformRadiusAboveTaper+p.platformRadiusBelowTaper)/2*p.platformLengthTaper;
    volume = M_PI*pow((p.platformRadiusAboveTaper+p.platformRadiusBelowTaper)/2,2)*p.platformLengthTaper;
    MonopileElement taperElement(p,loadContainer,platformBody,p.platformLengthTaper,A,B, crossSectionAreaTaper, volume );
    monopileElements.push_back(taperElement);
    qDebug() << "A.z: " << A.z();
    qDebug() << "B.z: " << B.z();
    A = B;

    qDebug()<< "element volume taper: " << volume;
    qDebug()<< "lengthOfElement taper: " << p.platformLengthTaper;
    qDebug()<< "crossSectionArea Taper: " << crossSectionAreaTaper;

    //Create element above taper
    double crossSectionAreaAboveTaper = 2*p.platformRadiusAboveTaper*p.platformLengthAboveTaper;
    volume = M_PI*pow(p.platformRadiusAboveTaper,2)*p.platformLengthAboveTaper;

    B = A+p.platformLengthAboveTaper*zAxisMonopile;
    MonopileElement aboveTaperElement(p,loadContainer,platformBody,p.platformLengthAboveTaper,A,B,crossSectionAreaAboveTaper,volume);
    monopileElements.push_back(aboveTaperElement);
    A = B;

    qDebug()<< "element volume Above taper: " << volume;
    qDebug()<< "lengthOfElement Above taper: " << p.platformLengthAboveTaper;
    qDebug()<< "crossSectionAreaAboveTaper: " << crossSectionAreaAboveTaper;

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

        MonopileElement monopileElement(p,loadContainer,platformBody,platformLengthOfElementBelowTaper,A,B,crossSectionAreaTowerElement ,volume);
        monopileElements.push_back(monopileElement);

        qDebug()<< "element volume tower: " << volume;
        qDebug()<< "z tower: " << z_tower;
        qDebug()<< "crossSectionArea tower:  " << crossSectionAreaTowerElement;

        A = B;
        z_tower = z_tower + towerLengthOfElement;
    }

    //Init Z damping force
    dragForceZBottom = std::make_shared<ChLoadBodyForce> (
      platformBody, //platformBody
      ChVector<>(0,0,0), //initialize
      true, //local force
      monopileElements.at(0).getMarker()->GetPos(), //bottom marker position is attack point of force
      true //local point
    );

    addedYawDampingTorque = std::make_shared<ChLoadBodyTorque>(
      platformBody,
      ChVector<>(0,0,0), //initialize
      true //local torque
    );

    loadContainer->Add(dragForceZBottom);
    loadContainer->Add(addedYawDampingTorque);
}

void Monopile::addMasses(ChSystem& system){

    ChVector<> vecGtoI = ChVector<>(0,0,p.distanceGtoE+p.platformLengthBelowTaper+p.platformLengthTaper+p.platformLengthAboveTaper);
    ChVector<> vecItoY = ChVector<>(0,0,p.towerHeight);
    ChVector<> vecYtoN = ChVector<>(-p.nacelleDistanceDownstream,0,p.nacelleDistanceToYawBearing);
    ChVector<> vecYtoH = ChVector<>(p.hubDistanceUpstream,0,p.hubDistanceToYawBearing);
    ChVector<> vecItoT = ChVector<>(0,0,p.towerCOGDistanceFromBottom);

    ChVector<> nacellePos = platformBody->TransformPointLocalToParent(vecGtoI+vecItoY+vecYtoN);
    nacelleBody = std::make_shared<ChBody>();
    nacelleBody->SetPos(nacellePos);
    nacelleBody->SetMass(p.nacelleMass);
    system.Add(nacelleBody);

    std::shared_ptr<ChLinkMateFix> nacelleConstraint = std::make_shared<ChLinkMateFix>();
    nacelleConstraint->Initialize(platformBody,nacelleBody);
    system.Add(nacelleConstraint);

    ChVector<> hubPos = platformBody->TransformPointLocalToParent(vecGtoI+vecItoY+vecYtoH);
    hubBody = std::make_shared<ChBody>();
    hubBody->SetPos(hubPos);
    hubBody->SetMass(p.hubMass);
    system.Add(hubBody);

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
        bladeConstraint->Initialize(platformBody,bladeBody);
        system.Add(bladeConstraint);

        qDebug()<< "creating blade nr:" << i;
        qDebug()<< "theta blade: " << theta;
        qDebug()<< "yBlade: " << yBlade;
        qDebug()<< "zBlade: " << zBlade;
    }
}

void Monopile::update(){
    //find out how many elements are submerged to distribute artificially added Damping among them
    int submergedElements = 0;
    for(auto &element : monopileElements){
        if(element.isSubmerged()){
            submergedElements++;
        }
    }

    double addedDampingXPerElement = p.addedDampingX/submergedElements;
    double addedDampingYPerElement = p.addedDampingY/submergedElements;
    double addedDampingZPerElement = p.addedDampingZ/submergedElements;

    CVector addedDamping = CVector(addedDampingXPerElement,addedDampingYPerElement,addedDampingZPerElement);

    //update elements
    for(auto &element : monopileElements){
        element.update(addedDamping);
    }

    ChVector<> markerVelocityAbs = monopileElements.at(0).getMarker()->GetAbsFrame().GetPos_dt();
    ChVector<> markerVelocityDir = platformBody->TransformDirectionParentToLocal(markerVelocityAbs);
    //calculate drag force on the bottom end of the cylinder
    double markerVelZ = markerVelocityAbs.z();

    qDebug() << "markerVelocity z" << markerVelZ;

    //simplification since the projection of the end of the zylinder on the xy plane will actually be an ellipsis
    double bottomEndAreaXY = M_PI*pow(p.platformRadiusBelowTaper,2);
    double dragForceZValue = 0;
    if(markerVelZ < 0){
        dragForceZValue = -0.5*p.rhoWater*p.dragCoefficientCylinderAxial*markerVelZ*bottomEndAreaXY;
    }
    else{
        dragForceZValue = 0;
    }
    //only apply damping force for a downwards motion of the platform
    dragForceZBottom->SetForce(ChVector<>(0,0,dragForceZValue),false);
    //qDebug() << "dragForceZBottom " << dragForceZBottom->GetForce().Length();

    //Calculate additional damping of platform around yaw-axis

    //Angular speed expressed in local coords
    ChVector<> platformAngularSpeed = platformBody->GetWvel_loc();

    double platformYawSpeed = platformAngularSpeed.z();
    ChVector<> torque = ChVector<>(0,0,-platformYawSpeed*p.addedDampingYaw);

    qDebug() << "yaw damping" << -platformYawSpeed*p.addedDampingYaw;

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
        //Draw Damping Force on the Bottom: light green
        glLineWidth(7);
        ChVector<> bottomPos = monopileElements.at(0).getMarker()->GetAbsCoord().pos;
        CVector bottomPosCVec = CVecFromChVec(bottomPos);
        qDebug() << dragForceZBottom->GetForce().Length();
        glColor4d(1,0.5,0,1);
        glVertex3d(bottomPosCVec.x,bottomPosCVec.y,bottomPosCVec.z);
        ChVector<> dragForceZBottomAbs = platformBody->TransformDirectionLocalToParent(dragForceZBottom->GetForce());
        CVector bottomForceEnd = CVecFromChVec(bottomPos+dragForceZBottomAbs*p.forceLineFactor);
//        qDebug() << "bottom drag force z:" << dragForceZBottom->GetForce().Length();
//        qDebug() << "bottomForceEnd x " << bottomForceEnd.x;
//        qDebug() << "bottomForceEnd y " << bottomForceEnd.y;
//        qDebug() << "bottomForceEnd z " << bottomForceEnd.z;
//        qDebug() << "bottomPos x" << bottomPos.x();
//        qDebug() << "bottomPos y" << bottomPos.y();
//        qDebug() << "bottomPos z" << bottomPos.z();
        glVertex3d(bottomForceEnd.x,bottomForceEnd.y,bottomForceEnd.z);
    glEnd();
    //Render Elements
    for(auto &element : monopileElements){
        element.render();
    }
}

