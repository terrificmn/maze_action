#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "maze_interfaces/action/maze.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <signal.h>

using namespace std::chrono_literals;   //ms 사용하기 위해

//사용자로부터 Goal을 받고, 이를 send
//string 타입의 feedback_msg를 지속적으로 출력해준다
class MazeActionClient : public rclcpp::Node {
private:
    rclcpp_action::Client<maze_interfaces::action::Maze>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<maze_interfaces::action::Maze>::SharedPtr goal_handle;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

protected:
    std::vector<int> turning_list;

public:
    MazeActionClient() : Node("maze_action_client") {
        this->action_client = rclcpp_action::create_client<maze_interfaces::action::Maze>(this, "maze");
        // timer_ = this->create_wall_timer(
        //         500ms, std::bind(&MazeActionClient::send_goal, this));
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MazeActionClient::send_goal, this));
        
        RCLCPP_INFO(this->get_logger(), "Action Client Node Created");
    }

    bool is_goal_done() const {
        return this->goal_done_;
    }

    bool is_goal_handle_none() {
        bool is_goal_handle = goal_handle == nullptr ? true : false;
        return is_goal_handle;
    }

    const std::shared_future<rclcpp_action::ClientGoalHandle<maze_interfaces::action::Maze>::WrappedResult> get_result_future() {
        auto result_future = this->action_client->async_get_result(goal_handle);
        return result_future;
    }

    const std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> get_cancel_result_future() {
        auto cancel_result_future = this->action_client->async_cancel_goal(goal_handle);
        return cancel_result_future;
    }

    void set_turning_list(std::vector<int> user_input_vec) {
        this->turning_list = user_input_vec;
    }

    std::vector<int> get_turning_list() {
        return this->turning_list;
    }

    void send_goal() {
        using namespace std::placeholders;

        //timer cancel required for send goal once
        this->timer_->cancel();

        this->goal_done_ = false;

        if (!this->action_client->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server is not available.");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = maze_interfaces::action::Maze::Goal(); 
        
        //send_goal()함수에 파라미터로 유저인풋을 넘길려고 했으나, walltimer에 type 때문에 에러
        //goal_msg.turning_sequence = this->turning_list;
        goal_msg.turning_sequence = this->get_turning_list();  //입력받은 배열 action msg에 넣어주기
        // 일단 send_goal() 호출되는 시점이 입력 받은 후 이기 떄문에, 일단 이렇게 처리

        auto send_goal_options = rclcpp_action::Client<maze_interfaces::action::Maze>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = std::bind(&MazeActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&MazeActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&MazeActionClient::result_callback, this, _1);

        auto goal_handle_future = this->action_client->async_send_goal(goal_msg, send_goal_options);
    }

    // 일단 feedback 부분에서도 type관련 에러인듯 하다... 일단 주석처리
    void feedback_callback(rclcpp_action::ClientGoalHandle<maze_interfaces::action::Maze>::SharedPtr, 
                            const std::shared_ptr<const maze_interfaces::action::Maze::Feedback> feedback) {
        //단순 출력
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->feedback_msg.c_str());
    }

    // goal_response_callback to check goal acceptation by action server:
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<maze_interfaces::action::Maze>::SharedPtr> future) {
        goal_handle = future.get();

        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, wait for result");
        }
    }

    // result_callback to get result once it is sent by action server:
    void result_callback(const rclcpp_action::ClientGoalHandle<maze_interfaces::action::Maze>::WrappedResult & result) {
        //RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback.feedback_msg);
        // 최종 result 여부 
        this->goal_done_ = true;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal not reached.. aborted");
                rclcpp::shutdown();
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal canceled");
                rclcpp::shutdown();
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        //나머지는 리턴 후
        if (result.result->success != false) {
            RCLCPP_WARN(this->get_logger(), "Congrats! Actions have been accompished!!");
        } 
        //RCLCPP_WARN_STREAM(this->get_logger(), "Result(cb) received message : " << result.result->success);
    }
};

// While문 입력에서 Ctrl C가 작동이 안되서 signal()사용하기 위한 함수
void signal_callback_handler (int signum) {
    std::cout << "Caught signal: Exit, signal_value=" << signum << std::endl;
    exit(signum);
}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    std::vector<int> user_inputs_vec;
    std::string user_input;

    // 입력 확인
    // if (argc !=2) {
    //     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
    //                 "Usage: choose one of number from 1 to 3");
    //     return 1;
    // }
    
    // Register signal and signal handler
    signal(SIGINT, signal_callback_handler);  //while에서 입력받을 때 ^C가 안먹혀서 signal() 넣어주기

    //객체만들기
    auto maze_action_client = std::make_shared<MazeActionClient>();
    
    // executor 잠깐 주석
    // Add node to executor
    //rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(maze_action_client);
    
    std::cout << "↑↑↑ If you go up straight, then press 3 ↑↑↑\n" 
            << "→→→ turn right, then press 2 →→→\n"
            << "←←← turn left, then press 0 ←←←\n"
            << "↓↓↓ go down, then press 1 ↓↓↓\n"
            << "        ↑↑↑ 3\n"
            << "←←← 0   ↓↓↓ 1   →→→ 2\n" 
            << "It's time to select numbers in order to find a way to a green box. \n";
    std::cout << "You should press 'q' to execute and finish the command." << std::endl;

    try {
        RCLCPP_INFO(maze_action_client->get_logger(), "Please enter number (0-3):");

        while(true) {
            try {
                // user 입력 받기
                std::cin >> std::ws >> user_input; //remove white space
            
                int convertedInput = atoi(user_input.c_str()); // converting string to integer due to 'q' exit
                // action_server에서 4 이상이 넘어가면 reject을 하지만, 10자리 이상 입력 방지
                if(convertedInput > 3) {
                    RCLCPP_WARN(maze_action_client->get_logger(), "Number exceeded. Only single digits are allowed. Retype");
                } else {
                    user_inputs_vec.push_back(convertedInput);
                }
                
                // q 입력시 종료
                if (user_input == "q") {
                    user_inputs_vec.pop_back();  //마지막 q로 들어간 값은 지우기
                    break;
                }

            } catch (rclcpp::exceptions::RCLError &e) {
                RCLCPP_ERROR(maze_action_client->get_logger(), "Error occured!");
                break;
            }
        }

        // for문은 분명이 작동하는데 for문 내에서 출력하려고 하면 원하는 결과가 안나온다. 
        // 다음 블럭의 RCLCPP_INFO가 먼저 출력되고 그 후에 출력이 됨
        // 그래서 RCLCPP_INFO가 for문이 돌기전에 출력이 되서 string에 넣어서 출력;;;
        std::string printInput;
        for(int num : user_inputs_vec) {
            printInput += std::to_string(num) + " ";  //convert from int to string
        }
        //입력 보여주기
        RCLCPP_INFO(maze_action_client->get_logger(), "Input sequence: %s", printInput.c_str()); //글자가 안깨지게 c_str()
        RCLCPP_INFO(maze_action_client->get_logger(), "Robot started to move");
        
    } catch (rclcpp::exceptions::RCLError &e) {
        RCLCPP_ERROR(maze_action_client->get_logger(), "Error occured!");
        return 1;
    }
    
    RCLCPP_INFO(maze_action_client->get_logger(), "==== Sending Goal... ====");
    
    //send_goal 한 번 한 후에 spin 함 비교해볼 것
    
    // send_goal에서 user_inputs_vec을 받아서 할려고 했는데 아규먼트가 create_wall_timer()만들때 맞지않는다고 나옴
    // 일단 우회적으로 set_turning_list 호출해서 입력받은것 저장해 놓고, send_goal()함수 내에서 처리
    // 현재 아래 잠깐 주석처리. 잘 되긴 함.. 마지막 result가 안되긴함
    // maze_action_client->set_turning_list(user_inputs_vec); 
    // maze_action_client->send_goal();
    
    // 위에서 executor로 추가한 노드로 바로 스핀
    //executor.spin();

    // 위의 try문이 끝나면 메소드 호출해서 유저 입력 등록하기 // send_goal()함수에 인자로 넘길려고 했으나 실패
    maze_action_client->set_turning_list(user_inputs_vec); 
    
    while(!maze_action_client->is_goal_done()) {
        rclcpp::spin_some(maze_action_client);
    }
    

    RCLCPP_ERROR(maze_action_client->get_logger(), "Action Client Ended");
    rclcpp::shutdown();

    return 0;
}