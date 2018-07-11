#include "PlatformParams.h"

using namespace chrono;

PlatformParams::PlatformParams(){

  distanceGtoE = 30.0845;

  //platformSetupPos = ChVector<>(0,0,-89.9155);
  platformSetupPos = ChVector<>(0,0,0);
  platformHeight = 130;
  platformMass = 7466330;
  platformMassMomentInertiaInRollAndPitch = 4229223000;//1.2622e10;
  platformMassMomentInertiaInYaw = 164230000;
  platformNrElementsBelowTaper = 1;

  platformLengthBelowTaper = 108;
  platformLengthTaper = 8;
  platformLengthAboveTaperInWater = 4;
  platformLengthAboveTaperAboveWater = 10;

  platformRadiusBelowTaper = 4.7;
  platformRadiusAboveTaper = 3.25;

  //Initial rotation of the monopile
  qRotationX = Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_X); //roll angle
  qRotationY= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Y); //pitch angle
  qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z); //yaw angle
  //Translate to initial Position

  zInit = -89.9155;

  initPosVec = ChVector<>(0,0,zInit);
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
  hubDiameter = 3;

  bladeNr = 3;
  bladeMass = 17740;
  bladeCOGDistanceFromRoot = 20.475;

  mooringLineNr = 3;
  mooringDiameter = 0.09;
  mooringStiffnessTimesLength = 384243000; //8471133.49;
  mooringUnstretchedLength = 902.2;
  mooringRaleyghDamping = 0.003000;
  mooringNrElements = 3;
  mooringRadiusToFairleadsFromCenter = 5.2;
  mooringRadiusToAnchorFromCenter = 853.87;
  mooringLineWeightInWaterPerMeter = 698.094;
  distanceZPlatformCOGtoFairlead = 19.9155;
  mooringPosAnchorZ = -320;
  mooringDensityPerUnit = 77.7066;

  //source: https:en.wikipedia.org/wiki/Drag_coefficient
  dragCoefficientCylinderAxial = 0.82;
  //source: http://sv.20file.org/up1/916_0.pdf
  //http://www-mdp.eng.cam.ac.uk/web/library/enginfo/aerothermal_dvd_only/aero/fprops/introvisc/node11.html
  dragCoefficientCylinderLateral = 1.20;

  addedDampingX = 100000;
  addedDampingY = 100000;
  addedDampingZ = 130000;
  addedDampingYaw = 13000000;

  seaLevel = 0; //sea level [m] in z-direction from origin
  rhoWater = 1025;
  g = 9.81;

  //visualisation params
  cSystemFactor = 50;
  dVectorFactor = 10;
  forceLineFactor = 0.005;
}
