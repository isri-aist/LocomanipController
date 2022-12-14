#pragma once

#include <set>

namespace LMC
{
/** \brief Hand. */
enum class Hand
{
  //! Left hand
  Left = 0,

  //! Right hand
  Right
};

namespace Hands
{
//! Both hands
const std::set<Hand> Both = {Hand::Left, Hand::Right};
} // namespace Hands

/** \brief Convert string to hand. */
Hand strToHand(const std::string & handStr);

/** \brief Get the opposite hand. */
Hand opposite(const Hand & hand);

/** \brief Get the sign of hand.

    Positive for left hand, negative for right hand.
*/
int sign(const Hand & hand);
} // namespace LMC

namespace std
{
/** \brief Convert hand to string. */
std::string to_string(const LMC::Hand & hand);
} // namespace std
