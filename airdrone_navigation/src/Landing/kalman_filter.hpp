
#include <Eigen/Dense>

#include <vision_msgs/msg/bounding_box2_d.hpp>

using Eigen::Matrix;

class KalmanFilter
{
    public:

        KalmanFilter()
        {
            Q = Matrix<float,10,10>::Identity() * 0.0001;

            A.block(0,0,5,5) = Matrix<float,5,5>::Identity();
            A.block(5,0,5,5) = Matrix<float,5,5>::Identity() * delta;
            A.block(0,5,5,5) = Matrix<float,10,10>::Zero();
            A.block(5,5,5,5) = Matrix<float,5,5>::Identity();

            H.block(0,0,5,5) = Matrix<float,5,5>::Identity();
            H.block(5,0,5,5) = Matrix<float,5,5>::Zero();
            
            R = Matrix<float,5,5>::Identity() * 0.01;

            I = Matrix<float,10,10>::Identity();

            X = Matrix<float,10,1>::Zero();
            X_hat = Matrix<float,10,1>::Zero();
            Y_hat = Matrix<float,5,1>::Zero();
            P_hat = Matrix<float,10,10>::Zero();
                        
        }

        void predict(vision_msgs::msg::BoundingBox2D bounding_box);
        vision_msgs::msg::BoundingBox2D update();

    private:

    vision_msgs::msg::BoundingBox2D filtered_box;

    Matrix<float,5,1>  Z;
    Matrix<float,10,1>  X;
    Matrix<float,10,1>  X_hat;
    Matrix<float,5,1>  Y;
    Matrix<float,5,1>  Y_hat;
    Matrix<float,10,10> P;
    Matrix<float,10,10> P_hat;
    Matrix<float,10,5> K;


    Matrix<float,10,10> Q;
    Matrix<float,10,10> A;
    Matrix<float,5,10> H;
    Matrix<float,5,5> R;
    Matrix<float,10,10> I;

    const float delta = 0.05;

};

void KalmanFilter::predict(vision_msgs::msg::BoundingBox2D bounding_box)
{
    float Xc, Yc, width, heigh, theta;

    Xc = bounding_box.center.x;
    Yc = bounding_box.center.y;
    width = bounding_box.size_x;
    heigh = bounding_box.size_y;
    theta = 0;

    Z(0,0) = Xc;
    Z(1,0) = Yc;
    Z(2,0) = width;
    Z(3,0) = heigh;
    Z(4,0) = theta;
    
    X_hat = A * X;

    P_hat = A * P * A.transpose() + Q;

    Y_hat = Z - H * X_hat;
}

vision_msgs::msg::BoundingBox2D KalmanFilter::update()
{
    K = P_hat * H.transpose() * (H * P_hat * H.transpose() + R).inverse(); 

    X = X_hat + K * Y_hat;

    P = (I - K * H) * P_hat;

    filtered_box.center.x = X(0,0);
    filtered_box.center.y = X(1,0);
    filtered_box.size_x = X(2,0);
    filtered_box.size_y = X(3,0);
}