#ifndef SOLVER_BASE_H
#define SOLVER_BASE_H

namespace constrained_ik
{

class SolverBase
{
  SolverBase();
  virtual ~SolverBase();
  
  virtual bool initialize();
  
};


} // namespace constrained_ik

#endif // SOLVER_BASE_H

