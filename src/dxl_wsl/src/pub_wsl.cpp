#include "rclcpp/rclcpp.hpp" // ROS2 C++ 클라이언트 라이브러리 헤더
#include "geometry_msgs/msg/vector3.hpp" // Vector3 메시지 타입 헤더
#include <termios.h> // 터미널 제어용 헤더 (키 입력 비동기 처리)
#include <unistd.h> // POSIX 운영체제 API 헤더
#include <fcntl.h> // 파일 제어 옵션 헤더
#include <memory> // 스마트 포인터 사용을 위한 헤더

// 키 입력이 있는지 비동기로 확인하는 함수
bool kbhit()
{
    struct termios oldt, newt; // 터미널 설정 구조체
    int ch; // 입력 문자 저장 변수
    int oldf; // 기존 파일 상태 플래그 저장 변수

    tcgetattr(STDIN_FILENO, &oldt); // 현재 터미널 설정 저장
    newt = oldt; // 새 설정 복사
    newt.c_lflag &= ~(ICANON | ECHO); // 캐노니컬 모드와 에코 끄기
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 새 설정 적용
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0); // 기존 파일 상태 플래그 저장
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 논블로킹 모드로 설정

    ch = getchar(); // 입력 문자 읽기

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 설정 복원
    fcntl(STDIN_FILENO, F_SETFL, oldf); // 원래 파일 상태 플래그 복원

    if(ch != EOF) // 입력이 있으면
    {
        ungetc(ch, stdin); // 입력 버퍼에 문자 다시 넣기
        return true; // 입력 있음 반환
    }

    return false; // 입력 없음 반환
}

// 키 입력을 1글자 읽는 함수 (에코 없이)
char getch()
{
    char buf = 0; // 읽은 문자 저장 변수
    struct termios old; // 터미널 설정 구조체
    memset(&old, 0, sizeof(old));  // 구조체 0으로 초기화

    fflush(stdout); // 출력 버퍼 비우기
    if (tcgetattr(STDIN_FILENO, &old) < 0) // 현재 터미널 설정 저장
        perror("tcgetattr()");
    old.c_lflag &= ~(ICANON | ECHO); // 캐노니컬 모드와 에코 끄기
    if (tcsetattr(STDIN_FILENO, TCSANOW, &old) < 0) // 새 설정 적용
        perror("tcsetattr ICANON");
    if (read(STDIN_FILENO, &buf, 1) < 0) // 1글자 읽기
        perror("read()");
    old.c_lflag |= (ICANON | ECHO); // 캐노니컬 모드와 에코 다시 켜기
    if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0) // 원래 설정 복원
        perror("tcsetattr ~ICANON");

    return buf; // 읽은 문자 반환
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = std::make_shared<rclcpp::Node>("dxl_pub_node"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS 설정 (최근 10개 메시지 유지)
    auto publisher = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile); // 퍼블리셔 생성

    geometry_msgs::msg::Vector3 vel; // 속도 메시지 객체 생성
    vel.x = 0; // x축 속도 초기화
    vel.y = 0; // y축 속도 초기화
    vel.z = 0; // z축 속도 초기화

    rclcpp::WallRate loop_rate(20.0); // 루프 주기 20Hz로 설정

    int vel1 = 0, vel2 = 0; // 현재 속도 변수
    int goal1 = 0, goal2 = 0; // 목표 속도 변수

    while (rclcpp::ok()) // ROS2가 동작 중일 때 반복
    {
        if (kbhit()) // 키 입력이 있으면
        {
            char c = getch(); // 입력 문자 1개 읽기
            switch(c) // 입력 문자에 따라 동작 결정
            {
                case 's': case ' ': // 's' 또는 스페이스바 입력 시
                    goal1 = goal2 = 0; // 목표 속도 0으로
                    vel1 = vel2 = 0;  // 현재 속도도 바로 0으로 (즉시 정지)
                    break;
                case 'f': goal1 = 50; goal2 = -50; break; // 'f' 입력 시 전진
                case 'b': goal1 = -50; goal2 = 50; break; // 'b' 입력 시 후진
                case 'l': goal1 = -50; goal2 = -50; break; // 'l' 입력 시 좌회전
                case 'r': goal1 = 50; goal2 = 50; break; // 'r' 입력 시 우회전
                default : goal1 = goal2 = 0; break; // 그 외 입력 시 정지
            }
        }

        // 목표 속도에 맞게 현재 속도를 부드럽게 변화 (가속/감속)
        if(goal1 > vel1) vel1 += 5; // 목표가 크면 증가
        else if(goal1 < vel1) vel1 -= 5; // 목표가 작으면 감소
        else vel1 = goal1; // 같으면 그대로

        if(goal2 > vel2) vel2 += 5; // goal2에 대해서도 동일하게 처리
        else if(goal2 < vel2) vel2 -= 5;
        else vel2 = goal2;

        vel.x = vel1; // 메시지에 x축 속도 대입
        vel.y = vel2; // 메시지에 y축 속도 대입
        vel.z = 0; // z축 속도는 0

        RCLCPP_INFO(node->get_logger(), "Publishing: x=%.2f, y=%.2f", vel.x, vel.y); // 현재 퍼블리시 값 출력
        publisher->publish(vel); // 토픽에 메시지 퍼블리시

        loop_rate.sleep(); // 다음 루프까지 대기 (20Hz)
    }

    rclcpp::shutdown(); // ROS2 종료
    return 0; // 프로그램 종료
}
