#include <mc_rtc/logging.h>

#include <LocomanipController/HandTypes.h>

using namespace LMC;

Hand LMC::strToHand(const std::string & handStr)
{
  if(handStr == "Left")
  {
    return Hand::Left;
  }
  else if(handStr == "Right")
  {
    return Hand::Right;
  }
  else
  {
    mc_rtc::log::error_and_throw("[strToHand] Unsupported Hand name: {}", handStr);
  }
}

Hand LMC::opposite(const Hand & hand)
{
  if(hand == Hand::Left)
  {
    return Hand::Right;
  }
  else // if(handStr == "Right")
  {
    return Hand::Left;
  }
}

int LMC::sign(const Hand & hand)
{
  if(hand == Hand::Left)
  {
    return 1;
  }
  else // if(handStr == "Right")
  {
    return -1;
  }
}

std::string std::to_string(const Hand & hand)
{
  if(hand == Hand::Left)
  {
    return std::string("Left");
  }
  else if(hand == Hand::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported hand: {}", std::to_string(static_cast<int>(hand)));
  }
}
