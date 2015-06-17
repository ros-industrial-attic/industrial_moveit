#ifndef CHAINTOOBSAVOIDJAC_HPP
#defineCHAINTOOBSAVOIDJAC_HPP

#include "frames.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"
#include "chain.hpp"

namespace constrained_ik
{
/**
 * @brief  Class to calculate the jacobian of a general
 * KDL::Chain, for obstacle avoidance
 */

class ChainToObsAvoidJac
{
public:
  explicit ChainToObsAvoidJac(const Chain& chain);
  ~ChainToObsAvoidJac();
  /**
   * Calculate the jacobian expressed in the base frame of the
   * chain, with reference to the link_point on link(link_number) of the chain
   * @param q_in input joint positions
   * @param jac output jacobian
   * @param link_number which link
   * @param link_point the x,y,z location of the point in TODO coordinates
   *
   * @return returns -1 when size of jac is not consistent 0 otherwise
   */
  int JntToJac(const JntArray& q_in, Jacobian& jac, int link_number, std::vector<double> link_point);
        
  int setLockedJoints(const std::vector<bool> locked_joints);
private:
  const Chain chain;
  Twist t_tmp;
  Frame T_tmp;
  std::vector<bool> locked_joints_;
  int nr_of_unlocked_joints_;
};
}
#endif
