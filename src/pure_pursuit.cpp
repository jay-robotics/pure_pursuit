#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
class purePursuit:public rclcpp::Node
{
public:
    purePursuit():Node("pure_pursuit")
    {
        cmd_vel_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        odom_subs_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&purePursuit::odom_callback, this, std::placeholders::_1));
        planned_path_subs_=this->create_subscription<nav_msgs::msg::Path>("/planned_path", 10, std::bind(&purePursuit::planned_path_callback, this, std::placeholders::_1));
        smooth_path_subs_=this->create_subscription<nav_msgs::msg::Path>("/smooth_path", 10, std::bind(&purePursuit::planned_path_callback, this, std::placeholders::_1));

        // ---- Debug / visualization publishers (view these in RViz) ----
        // transient_local + periodic republish so RViz always sees the path
        path_pub_=this->create_publisher<nav_msgs::msg::Path>(
            "/pure_pursuit/path", rclcpp::QoS(10).transient_local());
        goal_pub_=this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/goal", 10);
        robot_pub_=this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/robot", 10);
        lookahead_pub_=this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/lookahead", 10);
        actual_path_pub_=this->create_publisher<nav_msgs::msg::Path>("/pure_pursuit/actual_path", 10);

        publish_path();

        // Republish the path every 2s so RViz (even if started later) receives it
        path_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&purePursuit::publish_path, this));
    }
private:

    // std::vector< std::pair<double,double> > ={
    //     {0.0, 0.0},
    //     {1.0, 0.0},
    //     {2.0, 0.0},
    //     {3.0, 0.0},
    //     {4.0, 0.0},
    //     {5.0, 0.0},
    // };
    // std::vector< std::pair<double,double> > path={
    //     {0.0, 0.0},
    //     {1.0, 0.0},
    //     {2.0, 0.0},
    //     {3.0, 0.0},
    //     {4.0, 1.0},
    //     {5.0, 2.0},
    //     {7.0, 7.0},

    // };

    // S curve
    // std::vector<std::pair<double,double>> path = {
    // {0.0, 0.0},
    // {0.5, 0.0},
    // {1.0, 0.0},
    // {1.5, 0.1},
    // {2.0, 0.3},
    // {2.5, 0.6},
    // {3.0, 1.0},
    // {3.5, 1.5},
    // {4.0, 2.0},
    // {4.5, 2.4},
    // {5.0, 2.5},
    // {5.5, 2.4},
    // {6.0, 2.0},
    // {6.5, 1.5},
    // {7.0, 1.0},
    // {7.5, 0.5},
    // {8.0, 0.0}
    // };

    // 90 deg
    // std::vector<std::pair<double,double>> path = {
    // {0.0, 0.0},
    // {0.5, 0.0},
    // {1.0, 0.0},
    // {1.5, 0.0},
    // {2.0, 0.0},
    // {2.0, 0.5},pa
    // {2.0, 1.0},
    // {2.0, 1.5},
    // {2.0, 2.0},
    // {2.5, 2.0},
    // {3.0, 2.0},
    // {3.5, 2.0},
    // {4.0, 2.0}
    // };

    std::vector<std::pair<double,double>> path;


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


    // ---- Debug / visualization helpers (publish markers for RViz) ----
    // Publish the full reference path as a Path message (RViz: "Path" display)
    void publish_path()
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = this->now();
        for (const auto &p : path)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.first;
            ps.pose.position.y = p.second;
            ps.pose.orientation.w = 1.0;   // identity q
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

    void planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::vector<std::pair<double, double>> new_path;
        new_path.reserve(msg->poses.size());

        for (const auto &pose : msg->poses)
        {
            new_path.push_back({pose.pose.position.x, pose.pose.position.y});
        }

        if (new_path.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring planned_path with fewer than 2 poses");
            return;
        }

        path = new_path;
        current_segment = 0;
        publish_path();
        RCLCPP_INFO(this->get_logger(), "Received planned_path with %zu poses", path.size());
    }

    // Record the robot's current position into the trajectory trail and
    // publish the growing actual-path as a Path message (RViz: "Path" display).
    // Use a distinct topic so you can color it differently from the reference path
    // (pure pursuit cuts sharp corners, so compare the two to see the deviation).
    void record_actual_path(double x, double y)
    {
        // Only store a new point if the robot has actually moved (avoid duplicates)
        if (actual_path_.empty() ||
            getDistance(actual_path_.back().first, actual_path_.back().second, x, y) >= 0.03)
        {
            actual_path_.push_back({x, y});
        }
        publish_actual_path();
    }

    void publish_actual_path()
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = this->now();
        for (const auto &p : actual_path_)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.first;
            ps.pose.position.y = p.second;
            ps.pose.orientation.w = 1.0;   // identity q
            msg.poses.push_back(ps);
        }
        actual_path_pub_->publish(msg);
    }

    // Publish the selected lookahead (goal) point as a red sphere,
    // plus a red line from the robot to that goal.
    void publish_goal_marker(double gx, double gy, double rx, double ry)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = this->now();
        m.ns = "goal";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = gx;
        m.pose.position.y = gy;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.15;   // sphere diameter (m)
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
        goal_pub_->publish(m);

        // Red line from robot position -> selected goal
        visualization_msgs::msg::Marker line;
        line.header = m.header;
        line.ns = "goal_line";
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.02;                                       // line width
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; line.color.a = 1.0;
        line.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = rx; p1.y = ry; p1.z = 0.0;
        p2.x = gx; p2.y = gy; p2.z = 0.0;
        line.points.push_back(p1);
        line.points.push_back(p2);
        goal_pub_->publish(line);
    }

    // Publish the robot pose as a green arrow oriented with its yaw
    void publish_robot_marker(double rx, double ry, double yaw_rad)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = this->now();
        m.ns = "robot";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = rx;
        m.pose.position.y = ry;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.4; m.scale.y = 0.08; m.scale.z = 0.08;   // arrow length / width
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
        robot_pub_->publish(m);

        // Yellow sphere at the actual robot position (easier to spot)
        visualization_msgs::msg::Marker dot;
        dot.header = m.header;
        dot.ns = "robot_dot";
        dot.id = 1;
        dot.type = visualization_msgs::msg::Marker::SPHERE;
        dot.action = visualization_msgs::msg::Marker::ADD;
        dot.pose.position.x = rx;
        dot.pose.position.y = ry;
        dot.pose.orientation.w = 1.0;
        dot.scale.x = 0.08; dot.scale.y = 0.08; dot.scale.z = 0.08;
        dot.color.r = 1.0; dot.color.g = 1.0; dot.color.b = 0.0; dot.color.a = 1.0;
        robot_pub_->publish(dot);
    }

    // Publish the lookahead circle (radius = lookahead_distance) centred on the robot.
    // See which segments it intersects -> understand where the goal point comes from.
    void publish_lookahead_circle(double rx, double ry, double radius)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = this->now();
        m.ns = "lookahead_circle";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.02;                                       // line width
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.8;
        m.pose.orientation.w = 1.0;
        const int N = 60;
        for (int k = 0; k <= N; ++k)
        {
            double theta = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(N);
            geometry_msgs::msg::Point pt;
            pt.x = rx + radius * std::cos(theta);
            pt.y = ry + radius * std::sin(theta);
            pt.z = 0.0;
            m.points.push_back(pt);
        }
        lookahead_pub_->publish(m);
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
        //current pose
        double current_x=msg->pose.pose.position.x;
        double current_y=msg->pose.pose.position.y;

        // current orientation
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll,pitch,yaw;
        tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
        yaw=yaw*180.0/M_PI;
        if (yaw>180.0)
        {
            yaw-=360.0;
        }
        else if (yaw<-180)
        {
            yaw+=360;
        }


        double lookahead_distance=0.5;
        geometry_msgs::msg::TwistStamped cmd;

        if (path.size() < 2)
        {
            return;
        }

        // ---- Debug: publish robot pose + lookahead circle every tick ----
        publish_robot_marker(current_x, current_y, yaw * M_PI / 180.0);
        publish_lookahead_circle(current_x, current_y, lookahead_distance);
        // ---- Debug: log the actual trajectory the robot is tracing ----
        record_actual_path(current_x, current_y);

        for (int i=current_segment;i<path.size()-1;i++)                                       //becuase size and index are not equla ,if size is 3, then index will only till 0,1,2,3
        {
            double first_point_x=path[i].first;
            double first_point_y=path[i].second;
            double second_point_x=path[i+1].first;
            double second_point_y=path[i+1].second;

            //get points

            // printf("Checking segment %d : (%.2f, %.2f) -> (%.2f, %.2f)\n",i,first_point_x,first_point_y,second_point_x,second_point_y);
            auto points = get_intersection_points(
            first_point_x,
            first_point_y,
            current_x,
            current_y,
            second_point_x,
            second_point_y,
            lookahead_distance);

            // No intersection on this segment
            if (points.empty())
            {
                continue;
            }

            double goal_x, goal_y;

            // One intersection
            if (points.size() == 1)
            {
                goal_x = points[0].first;
                goal_y = points[0].second;
            }
            else
            {
                // Choose the point farther along the segment.
                double d1 = getDistance(first_point_x, first_point_y,
                                        points[0].first, points[0].second);

                double d2 = getDistance(first_point_x, first_point_y,
                                        points[1].first, points[1].second);

                if (d1 > d2)
                {
                    goal_x = points[0].first;
                    goal_y = points[0].second;
                }
                else
                {
                    goal_x = points[1].first;
                    goal_y = points[1].second;
                }
            }

            printf("Goal: %.2f %.2f\n", goal_x, goal_y);

            // ---- Debug: highlight the selected goal point (red sphere + red line) ----
            publish_goal_marker(goal_x, goal_y, current_x, current_y);

            // We found the first valid lookahead point.
            current_segment = i;

            // double current_distance_x=second_point_x-current_x;
            double current_distance_x_fn=getDistance(current_x,current_y,second_point_x,second_point_y);
            double remaining_dist=lookahead_distance-current_distance_x_fn;


            float linear_error=getDistance(current_x,current_y,goal_x,goal_y);
            double angle_to_goal=getAngle(current_x,current_y,goal_x,goal_y);
            double angle_error=angle_to_goal-yaw;
            if (angle_error>180)
            {
                angle_error-=360;
            }
            else if (angle_error<-180)
            {
                angle_error+=360;
            }
            printf("current x,y:%.2f,%.2f goalxy:%.2f,%.2f linearE:%.2f AngleE:%.2f\n",current_x,current_y,goal_x,goal_y,linear_error,angle_error);

            if (linear_error>0.05)
            {
                double p=0.5*angle_error;
                // double l=0.05*linear_error;
                cmd.twist.linear.x=0.5;
                cmd.twist.angular.z=p;
                // printf("Present:Current x,y:%.2f,%.2f | first_p x,y:%.2f,%.2f second_p x,y:%.2f,%.2f | Dis:%.2f | Rem:%.2f | final:x,y:%.2f,%.2f |Lerr:%.2f | Aerr:%.2f | Aspeed:%.2f goal:%.2f|yaw:%.2f \n",current_x,current_y, first_point_x,first_point_y, second_point_x,second_point_y, current_distance_x_fn, remaining_dist, final_x,final_y, linear_error, angle_error,p,angle_to_goal,yaw );
                cmd_vel_->publish(cmd);
            //     printf("breaking ");
                break;

            }
       }
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_subs_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smooth_path_subs_;
    // ---- Debug / visualization publishers ----
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;
    std::vector<std::pair<double,double>> actual_path_;   // trail the robot actually drove
    int current_segment=0;
};


int main(int argv, char *argc[])
{
    rclcpp::init(argv,argc);
    auto node=std::make_shared<purePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
