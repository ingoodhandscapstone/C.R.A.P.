#ifndef ACCEL_GRAVITY_MEASURE_MODEL_H
#define ACCEL_GRAVITY_MEASURE_MODEL_H

#include <Eigen/Core>
#include <InEKF/Core>

class AccelGravityMeasureModel : public InEKF::MeasureModel<InEKF::SE3<2,6>> {
    public:
        using typename InEKF::MeasureModel<InEKF::SE3<2,6>>::MatrixS;
        using typename InEKF::MeasureModel<InEKF::SE3<2,6>>::MatrixH;
        using typename InEKF::MeasureModel<InEKF::SE3<2,6>>::VectorV;
        using typename InEKF::MeasureModel<InEKF::SE3<2,6>>::VectorB;

        AccelGravityMeasureModel(double std = 1.0,
                                 const Eigen::Vector3d& gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

        void setNoise(double std);
        void setCovariance(const MatrixS& covariance);
        void setGravity(const Eigen::Vector3d& gravity);

        VectorB processZ(const Eigen::VectorXd& z, const InEKF::SE3<2,6>& state) override;

    private:
        Eigen::Vector3d gravity;
};

#endif
