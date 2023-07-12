#pragma once

#include <BaselineWalkingController/CentroidalManager.h>

namespace LMC
{
class LocomanipController;

/** \brief Centroidal manager.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
 */
class CentroidalManager : public BWC::CentroidalManager
{
public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManager(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

protected:
  /** \brief Const accessor to the controller. */
  const LocomanipController & ctl() const;

  /** \brief Accessor to the controller. */
  LocomanipController & ctl();
};
} // namespace LMC
