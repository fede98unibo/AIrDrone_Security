
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

#include <vision_msgs/msg/bounding_box2_d.hpp>

using Eigen::Matrix;

class KalmanFilter
{
    public:

        KalmanFilter() // Class Constructor: initialize all matrices
        {
            const float delta = 0.001;
            IDENTITY10 = Matrix<float,10,10>::Identity();
            IDENTITY5 = Matrix<float,5,5>::Identity();
            ZERO10 = Matrix<float,10,10>::Zero();
            ZERO5 = Matrix<float,5,5>::Zero();

            Q = IDENTITY10*0.0001;

            Matrix<float,5,5> A_top_right = delta*IDENTITY5;
            A << IDENTITY5, A_top_right, ZERO5, IDENTITY5; 

            H << IDENTITY5, ZERO5;

            R = IDENTITY5*0.01;

            // initialize other variables to ones to avoid zeros and NaN
            X = Matrix<float,10,1>::Ones();
            X_hat = Matrix<float,10,1>::Ones();
            Y = Matrix<float,5,1>::Ones();
            Y_hat = Matrix<float,5,1>::Ones();
            P = Matrix<float,10,10>::Ones();
            P_hat = Matrix<float,10,10>::Ones();
            K = Matrix<float,10,5>::Ones();
            
            std::cout << "All Kalman Filter variables initialized." << std::endl;        
        }

        vision_msgs::msg::BoundingBox2D predict(vision_msgs::msg::BoundingBox2D bounding_box);
        vision_msgs::msg::BoundingBox2D correct();
        void reset(); 


    private:

        // Class public variables
        Matrix<float,10,10> IDENTITY10;
        Matrix<float,5,5> IDENTITY5;
        Matrix<float,10,10> ZERO10;
        Matrix<float,5,5> ZERO5;

        Matrix<float,10,10> Q; // covariance matrix of the predicton error
        Matrix<float,10,10> A; // state transition matrix
        Matrix<float,5,10> H; // relationship btw states and observations
        Matrix<float,5,5> R; // uncertainty of the observations

        Matrix<float,5,1>  Z; // observable states
        Matrix<float,10,1>  X; // corrected states (observable and non-observable states)
        Matrix<float,10,1>  X_hat; // predicted states
        Matrix<float,5,1>  Y;
        Matrix<float,5,1>  Y_hat; // innovation
        Matrix<float,10,10> P; // correction of the state covariance matrix
        Matrix<float,10,10> P_hat; // state covariance matrix
        Matrix<float,10,5> K; // Kalman gain

};

// PREDICTION
vision_msgs::msg::BoundingBox2D KalmanFilter::predict(vision_msgs::msg::BoundingBox2D bounding_box)
{
    float Xc, Yc, width, heigh, theta;
    auto predicted_box = vision_msgs::msg::BoundingBox2D();

    Xc = bounding_box.center.x;
    Yc = bounding_box.center.y;
    width = bounding_box.size_x;
    heigh = bounding_box.size_y;
    theta = bounding_box.center.theta; //

    // populate vector Z
    Z(0,0) = Xc;
    Z(1,0) = Yc;
    Z(2,0) = width;
    Z(3,0) = heigh;
    Z(4,0) = theta;
    
    X_hat = A * X;

    P_hat = A * P * A.transpose() + Q;

    Y_hat = Z - H * X_hat;

    predicted_box.center.x = X_hat(0,0);
    predicted_box.center.y = X_hat(1,0);
    predicted_box.size_x = X_hat(2,0);
    predicted_box.size_y = X_hat(3,0);
    predicted_box.center.theta = X(4,0);

    return predicted_box;
}

// CORRECTION when a measurement is available
vision_msgs::msg::BoundingBox2D KalmanFilter::correct()
{

    auto filtered_box = vision_msgs::msg::BoundingBox2D();

    K = P_hat * H.transpose() * (H * P_hat * H.transpose() + R).inverse(); 

    X = X_hat + K * Y_hat;

    P = (IDENTITY10 - K * H) * P_hat;

    filtered_box.center.x = X(0,0);
    filtered_box.center.y = X(1,0);
    filtered_box.size_x = X(2,0);
    filtered_box.size_y = X(3,0);
    filtered_box.center.theta = X(4,0);

    return filtered_box;
}

// Reset matrices and vectors to ones
void KalmanFilter::reset() {
    X = Matrix<float,10,1>::Ones();
    X_hat = Matrix<float,10,1>::Ones();
    Y = Matrix<float,5,1>::Ones();
    Y_hat = Matrix<float,5,1>::Ones();
    P = Matrix<float,10,10>::Ones();
    P_hat = Matrix<float,10,10>::Ones();
    K = Matrix<float,10,5>::Ones();
}


#endif