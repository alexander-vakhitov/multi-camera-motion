//
// Created by alexander on 08.11.17.
//

#include <vector>
#include <multi-camera-motion/approx_relpose_generalized_fast.h>
#include "Eigen/Dense"

#include <iostream>

int main()

{
    Eigen::Quaterniond q(0.99,0.01,0.01,0.01);
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    R.setIdentity();
    Eigen::Vector3d t;
    t(2) = 1.0;
    t(1) = -1.0;
    //t.setZero();

    std::vector<Eigen::Matrix<double,6,6> > ws;
    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector3d p3d = Eigen::Vector3d::Random();
        Eigen::Vector3d c1 = Eigen::Vector3d::Random();
        Eigen::Vector3d c2 = Eigen::Vector3d::Random();
        Eigen::Vector3d dir1 = p3d-c1;
        dir1 = dir1/dir1.norm();
        Eigen::Vector3d n1 = dir1.cross(p3d);
        n1 = n1/n1.norm();
        Eigen::Vector3d dir2 = R*(p3d-c2);
        dir2 = dir2/dir2.norm();
        Eigen::Vector3d c2_loc = R*c2+t;
        Eigen::Vector3d n2 = dir2.cross(R*p3d+t);
        n2 = n2/n2.norm();
        Eigen::Matrix<double, 6, 1> u;
        u.block<3,1>(3,0) = n1;
        u.block<3,1>(0,0) = dir1;
        Eigen::Matrix<double, 6, 1> v;
        v.block<3,1>(3,0) = n2;
        v.block<3,1>(0,0) = dir2;
        Eigen::Matrix<double, 6, 6> w = u * (v.transpose());
        ws.push_back(w);
    }
    std::vector<Eigen::Vector3d> rsolns;
    approx_relpose_generalized_fast(ws[0],ws[1],ws[2],ws[3],ws[4],ws[5], rsolns);
    std::cout << rsolns.size() << std::endl;
//    std::cout << rsolns[0] << std::endl;
}