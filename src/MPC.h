#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct State{
vector<double> X;
vector<double> Y;
double A;
double Delta;

};

class MPC {
 public:


  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  State Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


};

#endif /* MPC_H */
