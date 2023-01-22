#pragma once

#include <TrajColl/CubicInterpolator.h>

#include <LocomanipController/HandTypes.h>

namespace LMC
{
class LocomanipController;
class ManipManager;

/** \brief Manipulation phase label. */
enum class ManipPhaseLabel
{
  //! Free phase
  Free = 0,

  //! Pre-reach phase
  PreReach,

  //! Reach phase
  Reach,

  //! Grasp phase
  Grasp,

  //! Hold phase
  Hold,

  //! Ungrasp phase
  Ungrasp,

  //! Release phase
  Release
};

namespace ManipPhase
{
/** \brief Base of manipulation phase. */
class Base
{
public:
  /** \brief Constructor.
      \param label manipulation phase label
      \param hand hand
      \param manipManager manipulation manager
  */
  Base(const ManipPhaseLabel & label, const Hand & hand, ManipManager * manipManager)
  : label_(label), hand_(hand), manipManager_(manipManager)
  {
  }

  /** \brief Get label. */
  virtual ManipPhaseLabel label() const
  {
    return label_;
  }

  /** \brief Run manipulation phase. */
  virtual void run() {}

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const
  {
    return false;
  }

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const
  {
    return nullptr;
  }

protected:
  /** \brief Const accessor to the controller. */
  const LocomanipController & ctl() const;

  /** \brief Accessor to the controller. */
  LocomanipController & ctl();

protected:
  //! Manipulation phase label
  ManipPhaseLabel label_;

  //! Hand
  Hand hand_;

  //! Manipulation manager
  ManipManager * manipManager_;
};

/** \brief Manipulation free phase. */
class Free : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Free(const Hand & hand, ManipManager * manipManager);
};

/** \brief Manipulation pre-reach phase. */
class PreReach : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  PreReach(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const override;

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const override;

protected:
  //! Phase end time [sec]
  double endTime_ = 0;
};

/** \brief Manipulation reach phase. */
class Reach : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Reach(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const override;

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const override;

protected:
  //! Function to interpolate reaching ratio
  std::shared_ptr<TrajColl::CubicInterpolator<double>> reachingRatioFunc_;
};

/** \brief Manipulation grasp phase. */
class Grasp : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Grasp(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const override;

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const override;
};

/** \brief Manipulation hold phase. */
class Hold : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Hold(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;
};

/** \brief Manipulation ungrasp phase. */
class Ungrasp : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Ungrasp(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const override;

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const override;
};

/** \brief Manipulation release phase. */
class Release : public Base
{
public:
  /** \brief Constructor.
      \param hand hand
      \param manipManager manipulation manager
  */
  Release(const Hand & hand, ManipManager * manipManager);

  /** \brief Run manipulation phase. */
  virtual void run() override;

  /** \brief Get whether manipulation phase is completed. */
  virtual bool complete() const override;

  /** \brief Make next manipulation phase. */
  virtual std::shared_ptr<Base> makeNextManipPhase() const override;

protected:
  //! Function to interpolate reaching ratio
  std::shared_ptr<TrajColl::CubicInterpolator<double>> reachingRatioFunc_;
};
} // namespace ManipPhase
} // namespace LMC

namespace std
{
/** \brief Convert manipulation phase label to string. */
std::string to_string(const LMC::ManipPhaseLabel & label);
} // namespace std
