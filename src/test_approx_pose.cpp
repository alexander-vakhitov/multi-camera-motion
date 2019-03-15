//
// Created by alexander on 08.11.17.
//

#include <vector>
#include <multi-camera-motion/approx_relpose_generalized_fast.h>
#include "Eigen/Dense"


#include <fstream>
#include <iostream>

int main1()

{
    Eigen::Quaterniond q(0.99,0.01,0.01,0.01);
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//    R.setIdentity();
    Eigen::Vector3d t;
    t(2) = 1.0;
    t(1) = -1.0;
    //t.setZero();

    std::vector<Eigen::Matrix<double,6,6> > ws;
    for (int i = 0; i < 6; i++)
    {
        // 3D point
        Eigen::Vector3d p3d = Eigen::Vector3d::Random();
        // Camera center in first observation
        Eigen::Vector3d c1 = Eigen::Vector3d::Random();
        // Camera center in second observation
        Eigen::Vector3d c2 = Eigen::Vector3d::Random();

        // image ray for first observation
        Eigen::Vector3d dir1 = p3d-c1;
        dir1 = dir1/dir1(2);
        Eigen::Vector3d n1 = c1.cross(dir1);

        // image ray for second observation
        Eigen::Vector3d dir2 = (R*p3d+t)-c2;
        dir2 = dir2/dir2(2);
        Eigen::Vector3d n2 = c2.cross(dir2);

        Eigen::Matrix<double, 6, 1> u;
        u.block<3,1>(0,0) = dir1;
        u.block<3,1>(3,0) = n1;
        std::cout << u.transpose() << "\n";
        Eigen::Matrix<double, 6, 1> v;
        v.block<3,1>(0,0) = dir2;
        v.block<3,1>(3,0) = n2;
        std::cout << v.transpose() << "\n";
        Eigen::Matrix<double, 6, 6> w = u * (v.transpose());
        ws.push_back(w);
    }
    std::vector<Eigen::Vector3d> rsolns;
    approx_relpose_generalized_fast(ws[0],ws[1],ws[2],ws[3],ws[4],ws[5], rsolns);
    std::cout << R << std::endl;
    std::cout << rsolns.size() << std::endl;
    std::cout << rsolns[0].transpose() << std::endl;
//    std::cout << rsolns[1].transpose() << std::endl;
}

int main()

{
    std::ifstream inp_file("/home/alexander/debug_ap_c", std::ios::in);
    std::vector<double> wvec;
    for (int i = 0; i < 216; i++)
    {
        double a;
        inp_file >> a;
        std::cout << a << std::endl;
        wvec.push_back(a);
    }


    std::vector<Eigen::Matrix<double,6,6> > wv;
    for (int i = 0; i < 6; i++)
    {
        int s = 36*i;
        Eigen::Matrix<double,6,6> w;
        int ind = 1;
        for (int cx = 0; cx < 6; cx++)
        {
            for (int cy = 0; cy < 6; cy++)
            {
                w(cx, cy) = wvec[s+ind];
                ind++;
            }
        }
        wv.push_back(w);
    }
    std::vector<Eigen::Vector3d> rsolns;
    approx_relpose_generalized_fast(wv[0], wv[1], wv[2], wv[3], wv[4], wv[5], rsolns);
    std::cout << rsolns.size() << std::endl;
}