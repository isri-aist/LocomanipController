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
  /** \brief Data of ext-ZMP (i.e., ZMP with external forces).


      The definition of ext-ZMP is given in the equation (9) of the paper:
        M Murooka, et al. Humanoid loco-Manipulations pattern generation and stabilization control. RA-Letters, 2021
  */
  struct ExtZmpData
  {
    //! Scale
    double scale = 1.0;

    //! Offset
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();

    /** \brief Convert conventional ZMP to ext-ZMP.
        \param zmp ZMP
     */
    inline Eigen::Vector2d apply(const Eigen::Vector2d & zmp) const
    {
      return scale * zmp - offset;
    }

    /** \brief Convert ext-ZMP to conventional ZMP.
        \param extZmp ext-ZMP
     */
    inline Eigen::Vector2d applyInv(const Eigen::Vector2d & extZmp) const
    {
      return (extZmp + offset) / scale;
    }
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerPreviewControlExtZmp(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger) override;

protected:
  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() override;

  /** \brief Calculate planned CoM acceleration.

      This method is overridden to support extended CoM-ZMP models (e.g., manipulation forces) in inherited classes.
  */
  virtual Eigen::Vector3d calcPlannedComAccel() const override;

  /** \brief Calculate reference data of MPC. */
  virtual Eigen::Vector2d calcRefData(double t) const override;

  /** \brief Calculate data of ext-ZMP. */
  ExtZmpData calcExtZmpData(double t) const;

protected:
  //! Data of ext-ZMP
  ExtZmpData extZmpData_;
};
} // namespace LMC
