#include "AccelGravityMeasureModel.h"

#include <stdexcept>

AccelGravityMeasureModel::AccelGravityMeasureModel(double std, const Eigen::Vector3d& gravity) {
    error_ = InEKF::ERROR::RIGHT;
    setGravity(gravity);
    setNoise(std);
}

void AccelGravityMeasureModel::setNoise(double std) {
    M_ = MatrixS::Identity() * std * std;
}

void AccelGravityMeasureModel::setCovariance(const MatrixS& covariance) {
    M_ = covariance;
}

void AccelGravityMeasureModel::setGravity(const Eigen::Vector3d& gravity) {
    this->gravity = gravity;
    VectorB b = VectorB::Zero();
    b.head(3) = gravity;
    setHandb(b);
}

AccelGravityMeasureModel::VectorB AccelGravityMeasureModel::processZ(const Eigen::VectorXd& z,
                                                                     const InEKF::SE3<2,6>& state) {
    (void)state;

    if (z.rows() < 3) {
        throw std::range_error("Wrong sized z");
    }

    VectorB zFull = VectorB::Zero();
    Eigen::Vector3d accel = z.head(3);

    const double norm = accel.norm();
    const double gravityNorm = gravity.norm();
    if (norm > 1e-9 && gravityNorm > 1e-9) {
        accel = accel * (gravityNorm / norm);
    }

    zFull.head(3) = accel;
    return zFull;
}
