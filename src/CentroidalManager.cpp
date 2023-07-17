#include <LocomanipController/CentroidalManager.h>
#include <LocomanipController/LocomanipController.h>

using namespace LMC;

CentroidalManager::CentroidalManager(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: BWC::CentroidalManager(ctlPtr, mcRtcConfig)
{
}

const LocomanipController & CentroidalManager::ctl() const
{
  return *static_cast<LocomanipController *>(ctlPtr_);
}

LocomanipController & CentroidalManager::ctl()
{
  return *static_cast<LocomanipController *>(ctlPtr_);
}
