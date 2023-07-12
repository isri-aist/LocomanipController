#include <mc_tasks/ImpedanceTask.h>

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

void CentroidalManagerPreviewControlExtZmp::runMpc()
{
  CentroidalManagerPreviewControlZmp::runMpc();

  ExtZmpData extZmpData = calcExtZmpData(ctl().t());
  plannedZmp_.head<2>() = (plannedZmp_.head<2>() + extZmpData.offset) / extZmpData.scale;
}

Eigen::Vector2d CentroidalManagerPreviewControlExtZmp::calcRefData(double t) const
{
  Eigen::Vector2d refZmp = CentroidalManagerPreviewControlZmp::calcRefData(t);
  ExtZmpData extZmpData = calcExtZmpData(t);
  return extZmpData.scale * refZmp - extZmpData.offset;
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
