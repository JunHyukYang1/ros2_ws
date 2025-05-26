#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory>

bool kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }

    return false;
}

char getch()
{
    char buf = 0;
    struct termios old;
    memset(&old, 0, sizeof(old));  // 모든 필드 0 초기화

    fflush(stdout);
    if (tcgetattr(STDIN_FILENO, &old) < 0)
        perror("tcgetattr()");
    old.c_lflag &= ~(ICANON | ECHO);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(STDIN_FILENO, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= (ICANON | ECHO);
    if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return buf;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dxl_pub_node");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto publisher = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);

    geometry_msgs::msg::Vector3 vel;
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    rclcpp::WallRate loop_rate(20.0); // 20 Hz

    int vel1 = 0, vel2 = 0;
    int goal1 = 0, goal2 = 0;

    while (rclcpp::ok())
    {
        if (kbhit())
        {
            char c = getch();
            switch(c)
            {
                case 's': case ' ':
                    goal1 = goal2 = 0;
                    vel1 = vel2 = 0;  // 바로 정지
                    break;
                case 'f': goal1 = 50; goal2 = -50; break;
                case 'b': goal1 = -50; goal2 = 50; break;
                case 'l': goal1 = -50; goal2 = -50; break;
                case 'r': goal1 = 50; goal2 = 50; break;
                default : goal1 = goal2 = 0; break;
}
        }

        // Accelerate or decelerate smoothly
        if(goal1 > vel1) vel1 += 5;
        else if(goal1 < vel1) vel1 -= 5;
        else vel1 = goal1;

        if(goal2 > vel2) vel2 += 5;
        else if(goal2 < vel2) vel2 -= 5;
        else vel2 = goal2;

        vel.x = vel1;
        vel.y = vel2;
        vel.z = 0;

        RCLCPP_INFO(node->get_logger(), "Publishing: x=%.2f, y=%.2f", vel.x, vel.y);
        publisher->publish(vel);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}