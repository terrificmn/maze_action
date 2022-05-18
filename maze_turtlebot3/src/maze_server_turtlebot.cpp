#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <map>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/action/maze.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// for image_sub // OpenCV
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/imgproc/imgproc.hpp"  //Include headers for OpenCV Image processing
#include "opencv2/highgui/highgui.hpp"  //Include headers for OpenCV GUI handling
#include "cv_bridge/cv_bridge.h"

// for euler function
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// GoalHandleMaze 사용
using GoalHandleMaze = rclcpp_action::ServerGoalHandle<custom_interfaces::action::Maze>;
using namespace std::chrono_literals;   //ms 사용하기 위해

// define each direction
std::vector<float> direction_flt_vec = {
    // (-1 * M_PI / 2),
    // M_PI,
    // M_PI / 2,
    // 0.0
    90 * (M_PI / 180),   //결국은 위의 원래 공식과 같은 ==
    M_PI,
    -90 * (M_PI / 180),
    0.0
};

// define each direction for users to be informed
std::vector<std::string> direction_str_vec = {"LEFT", "DOWN", "RIGHT", "UP"};


class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        using std::placeholders::_1;
        // 가제보 시뮬용   //"/cam0_sensor/image_raw" --터틀봇
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam0_sensor/image_raw", 10, 
            std::bind(&ImageSubscriber::image_callback, this, _1)
        );  
    }

    bool get_chk_if_green() {
        return this->is_green;
    }

protected:
    bool is_green;
    bool result_green;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv_bridge::CvImagePtr cvImgPtr;
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {     
        //RCLCPP_INFO(this->get_logger(), "started_image_callback");
            try {
                // convert ROS image msg to OpenCV image
                cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            } catch(cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            
            // 640 x 480
            // std::cout << "cols: " << cvImgPtr->image.cols << std::endl;  // width
            // std::cout << "rows: " << cvImgPtr->image.rows << std::endl;  // height
            int width = cvImgPtr->image.cols;
            int height = cvImgPtr->image.rows;
            int rgbPoint1[3], rgbPoint2[3], rgbPoint3[3]; //3곳의 RGB채널값 저장 array

            cv::Mat cvtImgToRgb; 
            // RGB 변환
            cv::cvtColor(cvImgPtr->image, cvtImgToRgb, CV_BGR2RGB);

            // at<cv::Vec3b>(row, col)
            // 첫 번째 포인트
            rgbPoint1[0] = cvtImgToRgb.at<cv::Vec3b>(height/2, width/2)[0];  //x, y 딱 중간  //Red channel
            rgbPoint1[1] = cvtImgToRgb.at<cv::Vec3b>(height/2, width/2)[1];  //Green channel
            rgbPoint1[2] = cvtImgToRgb.at<cv::Vec3b>(height/2, width/2)[2];  //Blue channel  // openCV BRG 상태라면 반대
            
            // 두 번째 포인트
            rgbPoint2[0] = cvtImgToRgb.at<cv::Vec3b>(200, 280)[0]; //(x, y) // x
            rgbPoint2[1] = cvtImgToRgb.at<cv::Vec3b>(200, 280)[1]; // y
            rgbPoint2[2] = cvtImgToRgb.at<cv::Vec3b>(200, 280)[2]; // z

            // 세 번째 포인트
            rgbPoint3[0] = cvtImgToRgb.at<cv::Vec3b>(200, 360)[0];  //x   //(x, y)
            rgbPoint3[1] = cvtImgToRgb.at<cv::Vec3b>(200, 360)[1];  //y
            rgbPoint3[2] = cvtImgToRgb.at<cv::Vec3b>(200, 360)[2];  //z

            this->img_green_check(&rgbPoint1[0], &rgbPoint2[0], &rgbPoint3[0], 3);  //주소값만 넘겨주기
    }

    bool img_green_check(int* spot1Ptr, int* spot2Ptr, int* spot3Ptr, int size) {
        // array주소값만 받아서 처리

        bool is_spot_green[3];
        
        // 더블pointer에 포인터를 담을 수 있게 또 포인터로 만들어준다
        int** rgbPointPPtr = new int*[size];  // pointer poninter 할당해주기 //allocate
        for(int i=0; i < size; i++) {
            rgbPointPPtr[i] = new int[size];
        }
        
        // referencing
        rgbPointPPtr[0] = spot1Ptr;
        rgbPointPPtr[1] = spot2Ptr;
        rgbPointPPtr[2] = spot3Ptr;
        
        for (int i=0; i < size; i++) {
            // 각각 포인트별로 Y(Green, 인덱스1)가 높으면 true주기   // indexing 0: r, 1: g, 2: b
            if ((rgbPointPPtr[i][1] > rgbPointPPtr[i][0]) && (rgbPointPPtr[i][1] > rgbPointPPtr[i][2])) {
                is_spot_green[i] = true;
            } else {
                is_spot_green[i] = false;
            }
        }

        //deallocate
        delete[] rgbPointPPtr;
        rgbPointPPtr = NULL;

        // 세곳의 포인트 모두 그린이 높으면 true 리턴
        if(is_spot_green[0] == true && is_spot_green[1] == true && is_spot_green[2] == true) {
            RCLCPP_WARN(this->get_logger(), "a high chance of green");
            return this->is_green = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "a low chance of green");
            return this->is_green = false;
        }
    }
};

////////// quaternion에서 euler로 conversion function /////////
//double odometryCallback_(const nav_msgs::msg::Odometry::SharedPtr quaternion) { //처음 형태 type을 맞춰야한다
double odometryCallback_(const geometry_msgs::msg::Quaternion quaternion) {
    // quaternion이 넘어옴

/*
이 부분이 잘 통과하기는 했는데, 함수에 매개변수로 넘겨주는 것이라, 기존의 예제처럼 SharedPtr 식으로 받으면 안됨
그냥 Quaternion으로 그대로 받아준다.
대신에 꼭 이 함수가 필요한지는 생각해볼 필요가 없다.. 어차피 odom_sub_cb() 메소드에서 
nav_msgs를 받으니 거기에서 그냥 pose.pose.position.x,y,z,w 몽땅 한번에 받아서 변환해도 될 것 같은데...
*/
    tf2::Quaternion q(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //return roll, pitch, yaw;
    //std::cout << "odom data processed" << std::endl;

    return yaw;
}



class MazeActionServer : public rclcpp::Node {
private:
    rclcpp_action::Server<custom_interfaces::action::Maze>::SharedPtr m_action_server;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    geometry_msgs::msg::Twist twist_msg; //for publish
    rclcpp::TimerBase::SharedPtr timer_;
    
protected:
    float yaw = 0.0;
    double forward_distance = 0.0;

public:
    MazeActionServer() : Node("maze_action_server") {
        using namespace std::placeholders;
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&MazeActionServer::laser_sub_cb, this, _1));
        
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MazeActionServer::odom_sub_cb, this, _1));
        
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        timer_ = create_wall_timer(
                100ms, std::bind(&MazeActionServer::publish_callback, this));

        this->m_action_server = rclcpp_action::create_server<custom_interfaces::action::Maze>(
            this,
            "maze",
            std::bind(&MazeActionServer::handle_goal, this, _1, _2),
            std::bind(&MazeActionServer::handle_cancel, this, _1),
            std::bind(&MazeActionServer::handle_accepted, this, _1));
            
            // std::bind(&MazeActionServer::handle_cancel, this, _1),
            // std::bind(&MazeActionServer::handle_accepted, this, _1));
            // 윗 부분 아마도 바꿔야할 듯.. execute_callback이 아니고 handle_goal 만들어야할지도
            // 파라미터를 handle_goal 메소드로 했을 때에는 _1, _2 이렇게 했었는데 차이점 알아보기

        RCLCPP_INFO(this->get_logger(), "===== Maze Action Server Started =====");
    }

    //handle_goal method (파라미터가 길어서 주의;;)
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, 
        std::shared_ptr<const custom_interfaces::action::Maze::Goal> goal) {
        
        (void)uuid;
        
        // 입력 받은 값(vector) 중 큰 값은 제외 후 reject
        for (int sequence : goal->turning_sequence) {
            if (sequence >= 4) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // CancelResponse
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_interfaces::action::Maze>> goal_handle) {
        
        RCLCPP_WARN(this->get_logger(), "Got request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_interfaces::action::Maze>> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, 
        // so spin up a new thread
        std::thread{std::bind(&MazeActionServer::execute, this, _1), goal_handle}.detach();
    }
    
    void laser_sub_cb(const sensor_msgs::msg::LaserScan::SharedPtr laserMsg) {
        if (isnan(laserMsg->ranges[1]) != 0) {
            this->forward_distance = 0.5;    // NaN 일 때 강제로 0.5로 넣어주기;; (좋은 방법은 아닌듯)
        } else {
            this->forward_distance = laserMsg->ranges[1];
        }
        //NaN이 아니면 0 return
        
        //this->forward_distance = laserMsg->ranges.size();
    }

    void odom_sub_cb(const nav_msgs::msg::Odometry::SharedPtr odomMsg) { 
        auto orientation = odomMsg->pose.pose.orientation;  
        
        //orientation이 orientation을 받으면서 quaternion으로 되버려서, odometryCallback_()함수와 자료형이 맞지 않는다.

        // conversion
        // 리턴값 확인하기!!, yaw값은 turn_robot() 메소드에서 사용된다
        this->yaw = odometryCallback_(orientation); 
    }

    void publish_callback() {
        this->cmd_vel_pub->publish(this->twist_msg);
    }

    // odom을 통해서 하기 때문에 nav_msgs_msg의 odometry를 가져온다
    // action의 turning_sequence의 값을 가져와야함
    void turn_robot(float defined_direction)  { // 사용자가 입력한 값 중에서 direction_flt_vec 에서 매치하는 데이터
        RCLCPP_INFO(this->get_logger(), "Robot Turns to %f", defined_direction); //예: PI 값

        float turn_offset = 100.0; //int >> float으로 바꿈

        //RCLCPP_INFO(this->get_logger(), "converted yaw is %f", this->yaw);

        while (std::abs(turn_offset) > 0.037) {  //0.087  //abs() for float,double <cmath>
            // P Gain Control, PID Control 이라고 한다고 함
            turn_offset = 0.5 * (defined_direction - this->yaw);  // 여기 값을 바꾸는 것은 의미가 없는 듯 하다..
            //odom_sub_cb에서 오일러각으로 변환된 yaw를 갱신, 현재 테스트로 yaw를 그냥 고정시켜야할 듯
            this->twist_msg.linear.x = 0.0;
            this->twist_msg.angular.z = turn_offset;

            //std::cout << "turn_offset: " << turn_offset << "  yaw: " << this->yaw << std::endl;
        }
        // 회전 이후는 로봇을 정지시킨다
        this->stop_robot();
    }

    void parking_robot() {
        //RCLCPP_INFO(this->get_logger(), "time start ");
        while (this->forward_distance > 0.28) {  // tight 0.23  // 0.28 due to camera's length
            this->twist_msg.linear.x = 0.04;  /// 0.05 넘어가도 꽤 빨라(?)진다 //turtlebot용
            this->twist_msg.angular.z = 0;
            
            ///확실히 여기에서는 문자열을 출력을 하면 터틀봇이3 조건이 false가 되지 않았는데도 빠져나가버림
            // RCLCPP_INFO or cout으로 출력을 안하면 조건에 맞게 계속 수행한다;;
            //RCLCPP_INFO(this->get_logger(), "forward distance: %f", this->forward_distance);
            
            
            /// 여기까지는 왼쪽 오른쪽 앞으로 즉, 0, 2, 3번 다 됨 레이저도 보고 멈춤
            ///중요~ 여기서는 확실하게 publish를 해줘야 한다~ 다른곳은 일단 안해도 되기는 하는 것 같음
            //여기를 퍼블리쉬를 안하면 센세데이터를 잘 못받아오는 것 같다
            //this->cmd_vel_pub->publish(this->twist_msg);
            //this->publish_callback();

            /// 이미 cmd_vel_pub 메소드에서 publish를 하는 중

            // 임의로 테스트중 무한루프이지만 subscribe를 안하면 큰 의미가 없는 듯 하다
            // 한번 돌고 끝남
        }
        // RCLCPP_INFO(this->get_logger(), "loop exit at %f", this->forward_distance);
        this->stop_robot();
    }
    
    void stop_robot() {
        //RCLCPP_INFO(this->get_logger(), "Robot stopped");
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.0;
    }

    
    void execute(const std::shared_ptr<GoalHandleMaze> goal_handle) {
        rclcpp::WallRate loop_rate(5);  
        const auto goal = goal_handle->get_goal();  //goal_handle->get_goal()->turning_sequence
        auto feedback = std::make_shared<custom_interfaces::action::Maze::Feedback>();
        auto result = std::make_shared<custom_interfaces::action::Maze::Result>();

        RCLCPP_INFO(this->get_logger(), "Executing goal");
        
        for (int val : goal->turning_sequence) {
            //RCLCPP_INFO(this->get_logger(), "Current Input: %d", val);

            RCLCPP_INFO(this->get_logger(), "Turning %s ", direction_str_vec[val].c_str()); //출력시 c_str() 글자 안깨짐

            // 사용자에게 보여주기 위한 vector 스트링, 일치하는 문자열 프린트
            feedback->feedback_msg = "Now.. turning " + direction_str_vec[val]; // 벡터에서 배열에 해당하는 문자열 합쳐주기
            RCLCPP_INFO(this->get_logger(), feedback->feedback_msg);  
            
            //메소드 호출 // 사용자 입력과 (turning_sequence) 같은 direction_flt_vec의 같은 배열 value값을 넘김
            turn_robot(direction_flt_vec[val]);

            parking_robot();
            // feedback 보냄
            goal_handle->publish_feedback(feedback);
        }

        // image_subscriber 객체 만들기-- spin_some 이 전혀 작동을 안함 executor와 관계가 있을까?
        // auto image_subscriber = std::make_shared<ImageSubscriber>();
        // //잘되는지 확인해보려면 spin()으로 테스트
        // // 테스트 후 spin_some()으로 바꿔야함
        // rclcpp::spin_some(image_subscriber); //spin once
        
        auto image_subscriber = std::make_shared<ImageSubscriber>();

        RCLCPP_INFO(get_logger(), "Please, wait for the result...");
        rclcpp::sleep_for(2000ms); //수행이 너무 빨라서 spin_some()하기 전에 sleep 걸기
        // spin_some()을 실행이 되면서 바로 아래줄이 수행되는 듯하다. 바로 get_chk_if_green()메소드를 호출해도
        // spin_some()이 아직 수행되고 있는 듯하다(?) 그래서 원하는 결과가 나오지를 않는다 (sleep_for를 안넣어주면 무조건 false)
        //rclcpp::spin_some(image_subscriber);
        rclcpp::spin_some(image_subscriber);
        
        bool is_green = image_subscriber->get_chk_if_green();
        
        // 최종 result 값 결정해서 보내주기
        if (!is_green) {
            RCLCPP_ERROR(get_logger(), "====== Failed ======");
            result->success = false;
            goal_handle->abort(result);
            
        } else {
            RCLCPP_INFO(get_logger(), "====== Succeed ======");
            result->success = true;
            goal_handle->succeed(result);
        }
    
        RCLCPP_INFO(get_logger(), "All goals have done.");
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto maze_action_server = std::make_shared<MazeActionServer>();
    
    try {
        // You MUST use the MultiThreadedExecutor to use, well, multiple threads
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(maze_action_server);
        
        executor.spin();  //반복
        //rclcpp::spin(maze_action_server);  // 그냥 spin으로 해도 결과는 같다;; ㅠ

    // exceptions 알아보기 추가로 더 알아보기
    } catch (rclcpp::exceptions::RCLError &e) {
        //signal(SIGINT, signal_handler);
        RCLCPP_INFO(maze_action_server->get_logger(), "Keyboard Interrupt (SIGINT)");
    }

    RCLCPP_WARN(maze_action_server->get_logger(), "Shutdown..");
    rclcpp::shutdown();
    
    return 0;
}
