#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
class purePursuit:public rclcpp::Node
{
public:
    purePursuit():Node("pure_pursuit")
    {
        cmd_vel_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        odom_subs_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&purePursuit::odom_callback, this, std::placeholders::_1));
    }
private:

    // std::vector< std::pair<double,double> > path={
    //     {0.0, 0.0},
    //     {1.0, 0.0},
    //     {2.0, 0.0},
    //     {3.0, 0.0},
    //     {4.0, 0.0},
    //     {5.0, 0.0},
    // };

    std::vector<std::pair<double,double>> path = {
    {0.0, 0.0},
    {4.0, 0.0},
    {4.0, 4.0},
    {0.0, 4.0},
    {0.0, 0.0}};

    // std::vector< std::pair<double,double> > path={
    //     {0.0, 0.0},
    //     {1.0, 0.0},
    //     {2.0, 0.0},
    //     {3.0, 0.0},
    //     {4.0, 1.0},
    //     {5.0, 2.0},
    //     {4.0, 4.0},
    // };

    // Get distance between two points
    double getDistance(double x1,double y1,double x2,double y2){
        return std::hypot(x2-x1, y2-y1);
    }
 
    // get angle between current and goal
    double getAngle(double x1, double y1, double x2, double y2){
        double dx=x2-x1;
        double dy=y2-y1;
        double angle_to_goal=(std::atan2(dy,dx))*180/M_PI;
        return angle_to_goal;
    }


    // Returns a vector of valid intersection points (0, 1, or 2 points)
    std::vector<std::pair<double, double>> get_intersection_points(double x1, double y1,  double xr, double yr, double x2, double y2, double L) 
        
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        
        double a = (dx * dx) + (dy * dy);
        double b = 2.0 * ((x1 - xr) * dx + (y1 - yr) * dy);
        double c = ((x1 - xr) * (x1 - xr)) + ((y1 - yr) * (y1 - yr)) - (L * L);
        
        double discriminant = (b * b) - (4.0 * a * c);
        std::vector<std::pair<double, double>> intersections;

        // Handle no intersection case safely
        if (discriminant < 0.0) {            //if <0 then no intersections, =0.0 then one intersection, >0 then two intersection
            // printf("Discriminant negative:%.2f No intersections: first x,y:%.2f,%.2f second x,y:%.2f,%.2f current:x,y %.2f,%.2f\n",discriminant,x1,y1,x2,y2,xr,yr);
            return intersections; 
        }

        double sqrt_disc = std::sqrt(discriminant);
        
        // BUG FIX 1: Added parentheses around the denominator (2.0 * a)
        double t1 = (-b + sqrt_disc) / (2.0 * a);
        double t2 = (-b - sqrt_disc) / (2.0 * a);

        // Check if t1 is within the line segment
        if (t1 >= 0.0 && t1 <= 1.0) {   //if 0<=t<=1 then the intersections lies on a line segment 
            double goal_x1 = x1 + t1 * dx;
            double goal_y1 = y1 + t1 * dy;
            intersections.push_back({goal_x1, goal_y1});
            // printf("")
        }
        
        // Check if t2 is within the line segment (and distinct if discriminant == 0)
        if (t2 >= 0.0 && t2 <= 1.0) {
            double goal_x2 = x1 + t2 * dx;
            double goal_y2 = y1 + t2 * dy;
            intersections.push_back({goal_x2, goal_y2});
            // std::cout << "t2 goal x,y: " << goal_x2 << ", " << goal_y2 << "\n";
        }
        // printf("\n");
        // printf("Segment: (%.2f,%.2f) -> (%.2f,%.2f)\n",
        //     x1, y1, x2, y2);

        // printf("disc = %.3f\n", discriminant);
        // printf("t1 = %.3f\n", t1);
        // printf("t2 = %.3f\n", t2);

        return intersections;
    }


    //cheeck if robo is between ans intersection
    double isBetween(double x1,double y1, double x, double y, double x2,double y2){
        double line_cross=(y2-y1)*(x-x1)-(x2-x1)*(y-y1);

        if (std::abs(line_cross)>0.05){
            return false;
        }
        bool within_x=(x >= std::min(x1,x2)-0.05) && (x <= std::max(x1,x2)+0.05);
        bool within_y=(y >= std::min(y1,y2)-0.05) && (y<= std::max(y1,y2)+0.05);         
        return within_x && within_y;
    }


    // std::pair<double,double> getLookaheadPoint(double x1,double y1, double x2, double y2, double d){

    //     //calculate segment length
    //     double L= std::sqrt(std::pow(x2-x1,2) + std::pow(y2-y1,2));

    //     if (L==0) return {x1,y1};

    //     double t=d/L;

    //     double x_new=x1+t*(x2-x1);
    //     double y_new=y1+t*(y2-y1);
    //     return {x_new,y_new};
    // }



    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        yaw = yaw * 180.0 / M_PI;
        if (yaw > 180.0) yaw -= 360.0;
        else if (yaw < -180) yaw += 360;

        double lookahead_distance = 1.0;
        geometry_msgs::msg::TwistStamped cmd;

        bool found_any = false;
        double goal_x = 0.0, goal_y = 0.0;
        int found_segment = current_segment;

        for (int i = current_segment; i < (int)path.size() - 1; i++)
        {
            double first_point_x = path[i].first;
            double first_point_y = path[i].second;
            double second_point_x = path[i + 1].first;
            double second_point_y = path[i + 1].second;

            auto points = get_intersection_points(
                first_point_x, first_point_y,
                current_x, current_y,
                second_point_x, second_point_y,
                lookahead_distance);

            if (points.empty())
            {
                continue;
            }

            double this_goal_x, this_goal_y;

            if (points.size() == 1)
            {
                this_goal_x = points[0].first;
                this_goal_y = points[0].second;
            }
            else
            {
                double d1 = getDistance(first_point_x, first_point_y, points[0].first, points[0].second);
                double d2 = getDistance(first_point_x, first_point_y, points[1].first, points[1].second);

                if (d1 > d2)
                {
                    this_goal_x = points[0].first;
                    this_goal_y = points[0].second;
                }
                else
                {
                    this_goal_x = points[1].first;
                    this_goal_y = points[1].second;
                }
            }

            // key change: don't break, just keep overwriting
            goal_x = this_goal_x;
            goal_y = this_goal_y;
            found_segment = i;
            found_any = true;
        }

        if (!found_any)
        {
            return;  // no intersection anywhere ahead, nothing to do this tick
        }

        current_segment = found_segment;

        float linear_error = getDistance(current_x, current_y, goal_x, goal_y);
        double angle_to_goal = getAngle(current_x, current_y, goal_x, goal_y);
        double angle_error = angle_to_goal - yaw;
        if (angle_error > 180) angle_error -= 360;
        else if (angle_error < -180) angle_error += 360;

        printf("current x,y:%.2f,%.2f goalxy:%.2f,%.2f linearE:%.2f AngleE:%.2f\n",
            current_x, current_y, goal_x, goal_y, linear_error, angle_error);

        if (linear_error > 0.05)
        {
            double p = 0.5 * angle_error;
            cmd.twist.angular.z = p;

            double abs_angle_error = std::abs(angle_error);
            double max_speed = 0.2;

            if (abs_angle_error > 45.0)
            {
                cmd.twist.linear.x = 0.05;
            }
            else
            {
                cmd.twist.linear.x = max_speed * (1.0 - (abs_angle_error / 45.0));
                if (cmd.twist.linear.x < 0.05)
                {
                    cmd.twist.linear.x = 0.05;
                }
            }

            cmd_vel_->publish(cmd);
        }
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs_;
    int current_segment=0;
};


int main(int argv, char *argc[])
{
    rclcpp::init(argv,argc);
    auto node=std::make_shared<purePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
