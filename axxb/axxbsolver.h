#ifndef AXXBSOLVER_H
#define AXXBSOLVER_H
#include <iostream>

// #include<Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include"type.h"
// #include<glog/logging.h>

//used for hand eye calibration
class AXXBSolver
{
public:
  AXXBSolver();
  AXXBSolver(const Poses A, const Poses B):A_(A),B_(B)
  {
    if(  !(A_.size()==B_.size()) ) {
        std::cerr <<"two sizes should be equal\n";
        exit(2);
    }


    if(  !(A_.size()>=2)  ) {
        std::cerr<<"at least two motions are needed\n";
        exit(2);
    }


  }

  virtual Pose SolveX()=0;

  Poses A_;
  Poses B_;
};

#endif // AXXBSOLVER_H
