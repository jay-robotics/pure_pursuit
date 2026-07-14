#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

class purePursuit:public rclcpp::Node
{
public:
    purePursuit():Node("pure_pursuit")
    {
        cmd_vel_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        odom_subs_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&purePursuit::odom_callback, this, std::placeholders::_1));
    }
private:

    std::vector< std::pair<double,double> > path={
        {0.0, 0.0},
        {1.0, 0.0},
        {2.0, 0.0},
        {3.0, 0.0},
        {4.0, 0.0},
        {5.0, 0.0},
    };

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        
        double current_x=msg->pose.pose.position.x;
        double current_y=msg->pose.pose.position.y;
        double current_angle_z=msg->pose.pose.orientation.z;
        double lookahead_distance=1.0;
        // int next_index=current_index+1;

        // int current_index=0;
        for (int i=0;i<=path.size()-1;i++)
        {
            double first_point_x=path[i].first;
            double first_point_y=path[i].second;
            double second_point_x=path[i+1].first;
            double second_point_y=path[i+1].second;
            if (current_x>=first_point_x && current_x<=second_point_x)
            {
                double current_distance_x=second_point_x-current_x;
                double remaining_dist=lookahead_distance-current_distance_x;
                double final_point=second_point_x+remaining_dist;
                printf("Present:Current x,y:%.2f,%.2f | first_p x,y:%.2f,%.2f second_p x,y:%.2f,%.2f | Dis:%.2f | Rem:%.2f | final:x,y:%.2f,%.2f \n",current_x,current_y,first_point_x,first_point_y,second_point_x,second_point_y,current_distance_x,remaining_dist,final_point,second_point_y);
            }
            else
            {
                printf("No:Current x,y:%.2f,%.2f first_p x,y:%.2f,%.2f second_p x,y:%.2f,%.2f\n",current_x,current_y,first_point_x,first_point_y,second_point_x,second_point_y);

            }
            
        }

    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs_;
};

int main(int argv, char *argc[])
{
    rclcpp::init(argv,argc);
    auto node=std::make_shared<purePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown;
}
