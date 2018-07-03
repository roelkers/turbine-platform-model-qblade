#include "PlatformParams.h"

using namespace chrono;

PlatformParams::PlatformParams(){

  distanceGtoE = -89.1955;

  platformSetupPos = ChVector<>(0,0,distanceGtoE);
  platformHeight = 130;
  platformMass = 7466330;
  platformMassMomentInertiaInRollAndPitch = 4229223000;//1.2622e10;
  platformMassMomentInertiaInYaw = 164230000;
  platformNrElementsBelowTaper = 15;

  platformLengthBelowTaper = 108;
  platformLengthTaper = 8;
  platformLengthAboveTaper = 14;

  platformRadiusBelowTaper = 4.7;
  platformRadiusAboveTaper = 3.25;

  //Initial rotation of the monopile
  qRotationX = Q_from_AngAxis(15 * CH_C_DEG_TO_RAD, VECT_X); //roll angle
  qRotationY= Q_from_AngAxis(15 * CH_C_DEG_TO_RAD, VECT_Y); //pitch angle
  qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z); //yaw angle
  //Translate to initial Position

  initPosVec = ChVector<>(0,0,0);
  initVelVec = ChVector<>(0,0,0);
  initAngVelVec = ChVector<>(0,0,0);

  nrElementsTower = 10;
  towerHeight = 77.6;
  towerMass = 249718;
  towerCOGDistanceFromBottom = 33.4;
  towerRadiusTop = 1.935;

  nacelleMass = 240000;
  nacelleDistanceDownstream = 1.9;
  nacelleDistanceToYawBearing = 1.75;

  hubMass = 56780;
  hubDistanceUpstream = 5;
  hubDistanceToYawBearing = 2.4;
  bladeMass = 17740;

  mooringLineNr = 0;
  mooringDiameter = 0.15;
  mooringStiffness = 84040;//8471133.49;
  mooringPreTensionForce = 89000;//case 1:89000; //case2: 623000
  mooringRaleyghDamping = 0.3000;
  mooringNrElements = 3;
  mooringAnchorRadiusFromFairlead = 900;
  mooringPosFairleadZFromBottom = 80;
  mooringPosAnchorZ = -320;
  mooringDensity = 2431;

  //source: https:en.wikipedia.org/wiki/Drag_coefficient
  dragCoefficientCylinderAxial = 0.82;
  //source: http://sv.20file.org/up1/916_0.pdf
  //http://www-mdp.eng.cam.ac.uk/web/library/enginfo/aerothermal_dvd_only/aero/fprops/introvisc/node11.html
  dragCoefficientCylinderLateral = 1.20;

  seaLevel = 0; //sea level [m] in z-direction from origin
  rhoWater = 1000;
  g = 9.81;

  //visualisation params
  cSystemFactor = 50;
  dVectorFactor = 10;
  forceLineFactor = 0.005;
}
