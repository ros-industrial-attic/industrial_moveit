#include "chaintoObsAvoidJac.hpp"

namespace constrained_ik
{
ChainToObsAvoidJac::ChainToObsAvoidJac(const Chain& _chain):
  chain(_chain),locked_joints_(chain.getNrOfJoints(),false),
  nr_of_unlocked_joints_(chain.getNrOfJoints())
{
}

ChainToObsAvoidJac::~ChainToObsAvoidJac()
{
}

int ChainToObsAvoidJac::setLockedJoints(const std::vector<bool> locked_joints)
{
  if(locked_joints.size()!=locked_joints_.size())
    return -1;
  locked_joints_=locked_joints;
  nr_of_unlocked_joints_=0;
  for(unsigned int i=0;i<locked_joints_.size();i++){
    if(!locked_joints_[i])
      nr_of_unlocked_joints_++;
  }
}

int ChainToObsAvoidJac::JntToJac(const JntArray& q_in, Jacobian& jac, int link_number, std::vector<double> link_point)
{
  if(q_in.rows()!=chain.getNrOfJoints())
    return -1;
  if(link_number != jac.columns())
    return -1;
  
  T_tmp = Frame::Identity();
  SetToZero(t_tmp);
  int j=0;
  int k=0;
  Frame total;
  for (unsigned int i=0;i<chain.getNrOfSegments();i++) 
  {
    //Calculate new Frame_base_ee
    if(chain.getSegment(i).getJoint().getType()!=Joint::None){
      //pose of the new end-point expressed in the base
      total = T_tmp*chain.getSegment(i).pose(q_in(j));
      //changing base of new segment's twist to base frame if it is not locked
      if(!locked_joints_[j])
        t_tmp = T_tmp.M*chain.getSegment(i).twist(q_in(j),1.0);
    }
    else
    {
      total = T_tmp*chain.getSegment(i).pose(0.0);
    }
    
    //Changing Refpoint of all columns to new ee
    changeRefPoint(jac,total.p-T_tmp.p,jac);

    //Only increase jointnr if the segment has a joint
    if(chain.getSegment(i).getJoint().getType()!=Joint::None){
      //Only put the twist inside if it is not locked
      if(!locked_joints_[j])
        jac.setColumn(k++,t_tmp);
      j++;
    }

    T_tmp = total;
  }
  return 0;
}
}
