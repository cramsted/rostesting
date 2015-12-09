#include "nav_gui.h"

namespace nav_gui
{

NavGUI::NavGUI():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<std::string>("nav_goal_topic", nav_goal_topic_, "nav_goal");
    nh_private_.param<std::string>("nav_state_topic", nav_state_topic_, "nav_state");
    nh_private_.param<std::string>("image_topic", image_topic_, "image");
    nh_private_.param<std::string>("gui_image_topic", gui_image_topic_, "gui_image");

    nav_goal_subscriber_ = nh_.subscribe(nav_goal_topic_, 1, &NavGUI::navGoalCallback, this);
    nav_state_subscriber_ = nh_.subscribe(nav_state_topic_, 1, &NavGUI::navStateCallback, this);
    image_subscriber_ = nh_.subscribe(image_topic_, 1, &NavGUI::imageCallback, this);
    gui_image_publisher_ = nh_.advertise<sensor_msgs::Image>(gui_image_topic_, 1);

    goal_.latitude = goal_.longitude = state_.base_latitude = state_.base_longitude = 0.0;
}

NavGUI::~NavGUI()
{}

void NavGUI::navGoalCallback(const rover_msgs::NavGoal &msg)
{
    goal_ = msg;
}

void NavGUI::navStateCallback(const rover_msgs::NavState &msg)
{
    state_ = msg;
}

double radians(double degrees){ return M_PI*degrees/180.0;  }

void NavGUI::imageCallback(const sensor_msgs::Image &msg)
{
    if((goal_.latitude != 0 || goal_.longitude != 0) && (state_.base_latitude != 0 || state_.base_longitude != 0))
    {
        cv_bridge::CvImagePtr cv_img_ptr;
        try
        {
            cv_img_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat cv_img = cv_img_ptr->image;
        int rows = cv_img.rows;
        int cols = cv_img.cols;

        double vect[2];
        vect[0] = EARTH_RADIUS * radians(goal_.latitude - state_.base_latitude) - state_.position[0];
        vect[1] = EARTH_RADIUS * cos(radians(state_.base_latitude)) * radians(goal_.longitude - state_.base_longitude) - state_.position[1];
        std::stringstream s;
        s << sqrt(vect[0]*vect[0] + vect[1]*vect[1]);
        cv::putText(cv_img, "s.str()", cv::Point(rows/2, cols/3), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,0,255));

        gui_image_publisher_.publish(cv_img_ptr->toImageMsg());
    }
}

}
