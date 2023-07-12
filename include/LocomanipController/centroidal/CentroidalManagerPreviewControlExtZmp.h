#pragma once

#include <BaselineWalkingController/centroidal/CentroidalManagerPreviewControlZmp.h>
#include <LocomanipController/CentroidalManager.h>

namespace LMC
{
/** \brief Centroidal manager with preview control.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
*/
class CentroidalManagerPreviewControlExtZmp : public CentroidalManager, BWC::CentroidalManagerPreviewControlZmp
{
public:
  /** \brief Data of ext-ZMP (i.e., ZMP with external forces). */
  struct ExtZmpData
  {
    //! Scale
    double scale = 1.0;

    //! Offset
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerPreviewControlExtZmp(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

protected:
  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() override;

  /** \brief Calculate reference data of MPC. */
  Eigen::Vector2d calcRefData(double t) const;

  /** \brief Calculate data of ext-ZMP. */
  ExtZmpData calcExtZmpData(double t) const;
};
} // namespace LMC
