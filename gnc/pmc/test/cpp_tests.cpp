#include <Eigen/Core>
#include <Eigen/SVD>
#include <iostream>


int main()
{
    Eigen::Matrix<float, 6, 12> thrust2forcetorque;
    Eigen::Matrix<float, 12, 6> forcetorque2thrust_;
    thrust2forcetorque.Random();
    forcetorque2thrust_ = Eigen::MatrixXf::Identity(12, 6);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(thrust2forcetorque, Eigen::ComputeThinU | Eigen::ComputeThinV);
    forcetorque2thrust_ = svd.solve(forcetorque2thrust_);
    std::cout << forcetorque2thrust_ << std::endl;
    return 0;
}