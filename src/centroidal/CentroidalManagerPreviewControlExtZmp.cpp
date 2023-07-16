#include <mc_tasks/ImpedanceTask.h>

#include <CCC/Constants.h>

#include <BaselineWalkingController/FootManager.h>

#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>
#include <LocomanipController/centroidal/CentroidalManagerPreviewControlExtZmp.h>

using namespace LMC;

CentroidalManagerPreviewControlExtZmp::CentroidalManagerPreviewControlExtZmp(LocomanipController * ctlPtr,
                                                                             const mc_rtc::Configuration & mcRtcConfig)
: BWC::CentroidalManager(ctlPtr, mcRtcConfig), LMC::CentroidalManager(ctlPtr, mcRtcConfig),
  BWC::CentroidalManagerPreviewControlZmp(ctlPtr, mcRtcConfig)
{
}

void CentroidalManagerPreviewControlExtZmp::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManagerPreviewControlZmp::addToLogger(logger);

  logger.addLogEntry(config_.name + "_ExtZmp_scale", this, [this]() { return extZmpData_.scale; });
  logger.addLogEntry(config_.name + "_ExtZmp_offset", this, [this]() { return extZmpData_.offset; });
}

void CentroidalManagerPreviewControlExtZmp::runMpc()
{
  extZmpData_ = calcExtZmpData(ctl().t());

  // Add hand forces effects
  plannedZmp_.head<2>() = extZmpData_.apply(plannedZmp_.head<2>());

  CentroidalManagerPreviewControlZmp::runMpc();

  // Remove hand forces effects
  plannedZmp_.head<2>() = extZmpData_.applyInv(plannedZmp_.head<2>());
}

Eigen::Vector3d CentroidalManagerPreviewControlExtZmp::calcPlannedComAccel() const
{
  // Replace plannedZmp_ with plannedExtZmp
  Eigen::Vector3d plannedExtZmp;
  plannedExtZmp << extZmpData_.apply(plannedZmp_.head<2>()), plannedZmp_.z();

  Eigen::Vector3d plannedComAccel;
  plannedComAccel << plannedForceZ_ / (robotMass_ * (mpcCom_.z() - refZmp_.z()))
                         * (mpcCom_.head<2>() - plannedExtZmp.head<2>()),
      plannedForceZ_ / robotMass_;
  plannedComAccel.z() -= CCC::constants::g;
  return plannedComAccel;
}

Eigen::Vector2d CentroidalManagerPreviewControlExtZmp::calcRefData(double t) const
{
  Eigen::Vector2d refZmp = CentroidalManagerPreviewControlZmp::calcRefData(t);
  ExtZmpData extZmpData = calcExtZmpData(t);
  // Add hand forces effects
  return extZmpData.apply(refZmp);
}

CentroidalManagerPreviewControlExtZmp::ExtZmpData CentroidalManagerPreviewControlExtZmp::calcExtZmpData(double t) const
{
  ExtZmpData extZmpData;
  extZmpData.scale = 0.0;
  extZmpData.offset.setZero();

  Eigen::Vector3d refZmp = ctl().footManager_->calcRefZmp(t);
  for(const auto & hand : Hands::Both)
  {
    if(ctl().manipManager_->manipPhase(hand)->label() != ManipPhaseLabel::Hold)
    {
      continue;
    }

    // Assume that objPoseOffset is constant
    sva::PTransformd objPose = ctl().manipManager_->objPoseOffset() * ctl().manipManager_->calcRefObjPose(t);
    sva::PTransformd handPose = ctl().manipManager_->config().objToHandTranss.at(hand) * objPose;
    // Represent the hand wrench in the frame whose position is same with the hand frame and orientation is same with
    // the world frame
    sva::ForceVecd handWrenchLocal = ctl().manipManager_->calcRefHandWrench(hand, t);
    sva::PTransformd handRotTrans(Eigen::Matrix3d(handPose.rotation()));
    sva::ForceVecd handWrench = handRotTrans.transMul(handWrenchLocal);

    const auto & pos = handPose.translation();
    const auto & force = handWrench.force();
    const auto & moment = handWrench.moment();

    // Equation (3) in the paper:
    //   M Murooka, et al. Humanoid loco-Manipulations pattern generation and stabilization control. RA-Letters, 2021
    extZmpData.scale -= force.z();
    extZmpData.offset.x() += (pos.z() - refZmp.z()) * force.x() - pos.x() * force.z() + moment.y();
    extZmpData.offset.y() += (pos.z() - refZmp.z()) * force.y() - pos.y() * force.z() - moment.x();
  }

  // Ignore the effect of CoM Z acceleration
  double mg = robotMass_ * mc_rtc::constants::gravity.z();
  extZmpData.scale /= mg;
  extZmpData.scale += 1.0;
  extZmpData.offset /= mg;

  return extZmpData;
}
