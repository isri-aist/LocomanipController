#include <LocomanipController/LocomanipController.h>
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
  // \todo Calculate ext-ZMP
  extZmpOffset_ = Eigen::Vector2d::Zero();
  extZmpScale_ = 1.0;

  CentroidalManagerPreviewControlZmp::runMpc();

  plannedZmp_.head<2>() += extZmpOffset_;
  plannedZmp_.head<2>() /= extZmpScale_;
}

Eigen::Vector2d CentroidalManagerPreviewControlExtZmp::calcRefData(double t) const
{
  Eigen::Vector2d refZmp = CentroidalManagerPreviewControlZmp::calcRefData(t);
  return extZmpScale_ * refZmp - extZmpOffset_;
}
