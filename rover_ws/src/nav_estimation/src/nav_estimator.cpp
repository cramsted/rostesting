#include "nav_estimator.h"
#include "tf/transform_datatypes.h"

namespace nav_estimator
{

NavEstimator::NavEstimator():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~")),
    xhat_p(7),
    P_p(7,7),
    Q_p(7,7),
    R_p(7,7),
    f_p(7),
    A_p(7,7),
    C_p(7),
    L_p(7)
{
    phat = 0;
    qhat = 0;
    rhat = 0;
    phihat = 0;
    thetahat = 0;
    psihat = 0;

    nh_private_.param<std::string>("gps_topic", gps_topic_, "fix");
    nh_private_.param<std::string>("imu_topic", imu_topic_, "imu");
    nh_private_.param<std::string>("state_topic", state_topic_, "nav_state");
    nh_private_.param<int>("iterations_per_prediction", N_, 10);
    nh_private_.param<float>("low_pass_filter", lpf_a_, 50.0);

    gps_subscriber_ = nh_.subscribe(gps_topic_, 1, &NavEstimator::gpsCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 1, &NavEstimator::imuCallback, this);
    state_publisher_ = nh_.advertise<rover_msgs::NavState>(state_topic_, 1);
}

NavEstimator::~NavEstimator()
{}

void NavEstimator::gpsCallback(const sensor_msgs::NavSatFix& msg)
{
    if(!gps_initialized_)
    {
        base_latitude_ = msg.latitude;
        base_longitude_ = msg.longitude;
        base_altidue_ = msg.altitude;

        initialize();
    }
    else
    {
        //update();
    }
}

float radians(float degrees)
{ return M_PI*degrees/180.0;  }

float degrees(float radians)
{ return 180.0*radians/M_PI;  }

void NavEstimator::initialize()
{
    //filter initialization
    xhat_p = Eigen::MatrixXf::Zero(7,7);
    xhat_p(2) = 9;
    P_p = Eigen::MatrixXf::Identity(7,7);
    P_p(0,0) = .03;
    P_p(1,1) = .03;
    P_p(2,2) = .01;
    P_p(3,3) = radians(5.0f);
    P_p(4,4) = .04;
    P_p(5,5) = .04;
    P_p(6,6) = radians(5.0f);
    gps_n_old = 0;
    gps_e_old = 0;
    gps_Vg_old = 0;
    gps_course_old = 0;
}

void NavEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
    float Ts = 10;
    alpha = exp(-lpf_a_*Ts);

    phat = alpha*phat + (1-alpha)*msg.angular_velocity.x;
    qhat = alpha*qhat + (1-alpha)*msg.angular_velocity.y;
    rhat = alpha*rhat + (1-alpha)*msg.angular_velocity.z;

    double phi, theta, psi;
    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(phi, theta, psi);
    phihat = phi;
    thetahat = theta;
    psihat = psi;

    prodiction(Ts);
}

void NavEstimator::prodiction(float Ts)
{
    float psidot, tmp, Vgdot;
    if(fabsf(xhat_p(2)) < 0.01f)
    {
        xhat_p(2) = 0.01;
    }

    for(int i=0;i<N_;i++)
    {
        psidot = (qhat*sinf(phihat) + rhat*cosf(phihat))/cosf(thetahat);
//        tmp = -psidot*Vahat*(xhat_p(4)*cosf(xhat_p(6)) + xhat_p(5)*sinf(xhat_p(6)))/xhat_p(2);
//        Vgdot = ((Vahat*cosf(xhat_p(6)) + xhat_p(4))*(-psidot*Vahat*sinf(xhat_p(6))) + (Vahat*sinf(xhat_p(6)) + xhat_p(5))*(psidot*Vahat*cosf(xhat_p(6))))/xhat_p(2);

        f_p = Eigen::VectorXf::Zero(7);
        f_p(0) = xhat_p(2)*cosf(xhat_p(3));
        f_p(1) = xhat_p(2)*sinf(xhat_p(3));
        f_p(2) = Vgdot;
        f_p(3) = GRAVITY/xhat_p(2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        f_p(6) = psidot;

        A_p = Eigen::MatrixXf::Zero(7,7);
        A_p(0,2) = cos(xhat_p(3));
        A_p(0,3) = -xhat_p(2)*sinf(xhat_p(3));
        A_p(1,2) = sin(xhat_p(3));
        A_p(1,3) = xhat_p(2)*cosf(xhat_p(3));
        A_p(2,2) = -Vgdot/xhat_p(2);
//        A_p(2,4) = -psidot*Vahat*sinf(xhat_p(6))/xhat_p(2);
//        A_p(2,5) = psidot*Vahat*cosf(xhat_p(6))/xhat_p(2);
        A_p(2,6) = tmp;
        A_p(3,2) = -GRAVITY/powf(xhat_p(2),2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        A_p(3,3) = -GRAVITY/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));
        A_p(3,6) = GRAVITY/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));

        xhat_p += f_p *(Ts/N_);
        P_p += (A_p*P_p + P_p*A_p.transpose() + Q_p)*(Ts/N_);
    }
}

}
