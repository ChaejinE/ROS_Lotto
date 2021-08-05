# Action
- Service는 동기적 요청/응답에 적합하다. 하지만 비동기적인 ROS topic이 필요한 경우에 적합한 것이 Action이다.
- ROS 책에서는 로봇의 goto\_position 이라는 함수를 예를 들고 있다.
  - 만약, 동기적인 통신을 하고 있다면 장거리를 이동하는 로봇이 도착할 때까지 로봇의 Position을 필요로 하는 프로그램(Node)는 정지한 상태가 된다.
  - 또한, 목적지로의 이동 을 취소하거나 변경할 수 없다.
  - 이러한 경우 과연 동기적인 통신방법이 적합할 것인가? 아니라는 것이 ROS의 대답이다.
- Action은 목표 지향적 행위의 인터페이스를 구현하는데 가장 좋은 방법이라고 한다.
- 요청 & 응답처럼 **행위를 시작**하기 위한 Goal(목표)를 사용한다.
- **행위가 완료**되면 Result(결과)를 보낸다.
- 더 나아가 **진행 정보 갱신**을 제공하기 위해 Feedback을 사용한다.
- 목표가 취소되거나 변경되는 것 또한 허용한다.
- 본질적으로 토픽의 집합(목표, 결과, 피드백 등)을 짝지어 사용하는 방법을 기술하는 **보다 높은 수준의 프로토콜**인 것이다.
# Action의 정의
- .action 파일 내의 각 필드는 자신만의 메시지가 된다.
- Timer.action 이라는 액션파일을 정의해보자.
```shell
# goal
duration time_to_wait # 대기하기 원하는 시간
---
# result
duration time_elapsed # 대기 시간
uint32 updates_sent # 진행되는 동안 갱신 갯수
---
# feedback
duration time_elapsed # 시작 부터 경과된 시간
duration time_remaining # 종료될 때까지 남은 시간
```
- 정의영역 구분자로 세개의 ---(대시기호)를 사용한다.
- Action을 사용해서 상호 작용할 때 사용할 코드와 클래스 정의를 생성하기 위해 catkin\_make를 반드시 해줘야한다.
## 1. CMakeLists.txt
```shell
find_package(catkin ...
  ...
  actionlib_msgs
)
```
- 다른 패키지와 함께 find\_package에 actionlib\_msgs를 추가한다.
```shell
add_action_files(
  DIRECTORY action
  FILES Timer.action
)
```
- 컴파일 하길 원하는 action 파일을 catkin에 알리기 위해 add\_action\_files를 호출하도록 한다.
```shell
generate_mesagges(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)
```
- 액션이 올바르게 컴파일 될 수 있도록 actionlib\_msgs를 의존성으로서 명시적으로 나열한다.
```shell
catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
)
```
- actionlib\_msgs를 catkin에 대한 의존성으로 추가한다.
## 2. package.xml
- **actionlib & actionlib\_msgs**를 build\_depend, exec\_depend에 추가한다.
  - build\_depend : 컴파일 시 의존성
  - exec\_depend : 실행 시 의존성
## 3. catkin\_make
- TimerAction.msg, TimerActionFeedback.msg, TimerActionGoal.msg, TimerActionResult.msg, TimerFeedback.msg, TimerGoal.msg, TimerResult.msg를 생성해준다.
- 위 메시지들은 ROS 토픽상에 만들어져 액션 클라이언트와 서버 프로토콜을 구성하는데 사용된다.
- 생성된 메시지 정의가 차례대로 메시지 생성기에 의해 처리되어 해당하는 클래스 정의 파일을 생성한다.
## 4. Basic Action Server
```python
import rospy

import time
import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult

def do_timer(goal):
    start_time = time.time()
    time.sleep(goal.time_to_wait.to_sec())
    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = 0
    server.set_succeeded(result)

rospy.init_node("timer_action_server")
server = actionlib.SimpleActionServer("timer", TimerAction, do_timer, False)
server.start(0)
rospy.spin()
```
- simple\_action\_server.py
- actionlib : SimpleActionServer 제공
- 몇몇 생성된 메시지 클래스를 임포트
- goal : 자료형이 TimerGoal인 인자, Timer.action의 목표 영역에 해당한다.
- result : TimerReulst 자료형인 결과 메시지, ros의 duration 자료형으로 변환해서 time\_elapsed 필드에 저장하고 있다.
- set\_succeeded(result) : 결과를 전송 -> 목표를 성공적으로 달성했음을 SimpleActionServer에 알린다. 실패하는 경우도 있다고한다.
- SimpleActionServer : 토픽이 광고될 이름공간 결정할 서버이름, 액션의 자료형, 목표 콜백, 서버의 자동시작 방지 flag
  - 서버 자동시작이 False이므로 start를 명시적으로 실행해주고 spin 루프로 들어가게한다.
  - 서버 자동시작 비활성화는 경쟁상태(race condition) 문제를 위함이다.
## 5. 예상 결과 확인
```shell
rosrun basics simple_action_server.py
```
```shell
rostopic list
```
```shell
/rosout
/rosout_agg
/timer/cancel
/timer/feedback
/timer/goal
/timer/result
/timer/status
```
- timer 이름공간에 액션을 관리하기 위한 다섯 개 토픽이 있었다.
```shell
rostopic info /timer/goal
```
```shell
Type: basics/TimerActionGoal

Publishers: None

Subscribers: 
 * /timer_action_server (http://10.0.1.83:37557/)
```
- /timer/goal 토픽에 대한 정보를 확인해봤다. TimerActionGoal 타입은 뭘까?
```shell
rosmsg show TimerActionGoal
```
```shell
[basics/TimerActionGoal]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
basics/TimerGoal goal
  duration time_to_wait
```
- goal.time\_to\_wait 필드처럼 목표가 정의 된 것을 찾을 수 있다.
- 명시하지 않은 다른 추가적 필드도 있음을 확인할 수 있다. 이러한 Bookkeeping 정보는 서버 코드가 목표 메시지를 보기전에 자동으로 삭제가 된다고한다.
- 즉, 목표 실행에서 볼 수 있는 것은 .action 파일에서 정의한 TimerGoal 메시지다.
- 사용자가 정의한 Timer.msg는 TimerGoal, TimerReulst, TimerFeedback 자료형 외에 Action이라는 이름을 가진 다른 자료형들이 추가되어있는데, catkin\_ws/devel/share/basics/msg 디렉토리에서 확인할 수 있다.
  - actionlib에 있는 라이브러리를 사용하면 Action과 함꼐 자동 생성된 메시지를 사용할 필요가 없다.
  - 대부분 Goal, Result, Feedback 메시지 만으로 충분하다고한다.
- Action은 ROS 메시지를 기반으로 만들어진 고수준 프로토콜이라고 할 수 있다. 하지만 대부분 application에서는 actionlib 라이브러리가 시스템 안에서 메시지를 처리하면서 일을 해준다.

## 5. Action 사용
- simple\_action\_client.py
```python
import rospy

import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult

rospy.init_node("timer_action_client")
client = actionlib.SimpleActionClient("timer", TimerAction)
client.wait_for_server()
goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)
client.send_goal(goal)
client.wait_for_result()
print("Time elapsed: %f"%(client.get_result().time_elapsed.to_sec()))
```
- SimpleActionClient 클래스를 통한 방법이 가장쉽다.
  - 첫 번째 생성자 인자는 액션 클라이언트의 이름이며 서버와 통신할 때 사용하는 토픽을 결정하는 데 사용한다.
  - 즉, 액션 클라이언트 이름은 서버 이름과 같아야한다.
  - 두 번째 인자는 액션의 자료형으로 서버와 일치해야한다.
- 액션 서버의 동작 여부는 이전 본것처럼 다섯 개의 광고된 토픽을 확인한다.
- client.wait\_for\_result() : rospy.wait\_for\_service()와 마찬가지로 서버가 준비될 때까지 차단된다.
- goal.time\_to\_wait : 타이머가 대기해야하는 목표를 송신한다.
  - client.send\_goal(goal) 로 송신
  - 정상적으로 실행된다면 5초간 여기서 멈춘다.
- Reulst 도착 시 클라이언트 객체로부터 결과를 검색해서 정의한 time\_elapsed 필드를 출력하게 된다.

## 6. Client 예상 결과 확인
```shell
rosrun basics simple_action_client.py
```
```shell
Time elapsed: 5.002919
```

## 7. 보다 복잡한 Action Server
- fancy\_action\_server.py
```shell
import rospy

import time
import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def do_timer(goal):
    start_time = time.time()
    update_count = 0

    if goal.time_to_wait.to_sec() > 60.0:
        result = TimerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_aborted(result, "Timer aborted due to too-long wait")
        return

    while (time.time() - start_time) < goal.time_to_wait.to_sec():
        if server.is_preempt_requested():
            result = TimerResult()
            result.time_elapsed = \
                    rospy.Duration.from_sec(time.time() - start_time)
            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return

        feedback = TimerFeedback()
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed
        server.publish_feedback(feedback)
        update_count += 1

        time.sleep(0.1)

    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "Timer completed successfully")


rospy.init_node("timer_action_server")
server = actionlib.SimpleActionServer("timer", TimerAction, do_timer, False)
server.start()
rospy.spin()
```
- update\_count : feedback을 몇번 발행하는지 추적하는 변수
- set\_aborted() : 60초 이상으로 goal을 설정했으면, goal을 명시적으로 중지한다. 메시지도 함께 보내서 중단됐음을 알린다.
  - set\_succeeded() 처럼 result를 포함한다.
  - 단순 오류 처리 코드를 위함으로 보인다.
- while문 : 요청 시간 동안 한 번에 일시 정지하는 대신에 조금씩 일시 정지하면서 while 반복문을 돈다.
- is\_preempt\_requested() : 선점을 확인한다. client가 목표 추적을 멈추도록 요청했으면 True를 반환한다.
  - 다른 client가 새로운 목표를 보냈을 때도 일어날 수 있다.
- set\_preempted() : 결과를 채우고 status 를 문자열로 제공한다.
- TimerFeedback 자료형을 사용해서 피드백을 보낸다. time\_elapsed, time\_remaining 필드를 채운다.
- publish\_feedback() : client로 채운 필드를 보낸다.
