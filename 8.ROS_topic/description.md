# Topic
- Node는 그 자체로는 쓸모 없다. 하지만, Node가 정보와 데이터를 교환하면서 Another Node 와 통신할 때 유용하게 된다.
- 가장 보편적인 통신 방법은 **Topic** 이다.
- Topic은 **정의된 자료형**을 가지는 메세지 스트림이다.
  - ex) Image 메시지 자료형의 image topic
  - ex) LaserScan 메시지 자료형의 scan topic
- Publish/Subscribe 통신기법을 사용한다.
  - 분산 시스템에서 보편적 방법 중 하나
- 먼저 Advertise하고나서 실제 데이터를 해당 Topic으로 Publishing 할 수 있다.
- 특정 Topic Message를 수신하기 원하는 Node는 roscore에게 요청함으로써 Subscribing 할 수 있다.
  - 요청하면 특정 Topic은 요청한 노드로 전달된다.
## Topic Publishing
```python
import rospy
from std_msgs.msg import Int32

rospy.init_node("topic_publisher")
pub = rospy.Publisher("counter", Int32)
rate = rospy.Rate(2)

count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
```
- 보통 쉬뱅으로 인터프리터 지시어를 붙여준다.
  - 운영체제로 하여금 이 파일이 파이썬 파일이며, 파이썬 인터프리터로 전달되도록 한다.
  - #!/usr/bin/env python
```shell
chmod u+x topic_publisher.py
```
- 실행 권한 부여
```python
from std_msgs.msg imprt Int32
```
- std\_msgs : ROS 표준 메시지 패키지
- Int32 : 32비트 정수
- .msg에 패키지 정의가 저장되어있다.
- 다른 패키지에 있는 메시지를 사용하므로 package.xml 파일에 의존성을 추가해서 **ROS build 시스템에 알려준다.**
```xml
<depend package="std_msgs" />
```
- 이러한 의존성 없이는 ROS는 메시지 정의를 찾을 수 없다.
```python
rospy.init_node("topic_publisher")
```
- Node를 초기화한다.
```python
pub = rospy.Publisher("counter", Int32)
```
- Publisher를 통해 Topic을 Advertise 한다
- Topic에 counter 라는 이름을 부여하고, 자료형이 Int32임을 알려준다.
- Publisher가 roscore에 연결을 설정하고 관련정보를 보내준다.
- 다른 노드가 counter Topic을 Subscribing하려고 할 때 **roscroe는 Publisher와 Subscriber 목록을 공유한다.**
  - Node 들은 이 목록을 각 Topic의 Publisher와 Subscriber 사이를 직접 연결하는데 사용한다.
- 이 시점에 Topic은 Advertise 되어서 다른 Node에서 Subscribe할 수 있게 된다
  - 이 Topic을 통해 메시지를 실제로 Publish할 수 있게 된다.
```python
rate = rospy.Rate(2)
```
- 발행 속도를 Hz 단위로 설정
- 초당 두 번 Publish
```python
count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
```
- is\_shutdown() : 노드가 종료될 상황이면 True / 아니면 False
- while문 안에서 count를 publish 하고 1씩 증가시킨다.
- rate.sleep() : 대략 2Hz주기로 while문 몸체를 실행할 수 있도록 일시 정지
- 지금 까지 토픽을 광고하고, 이 토픽을 통해 정수를 발행하는 ROS Node를 만드는 연습이었다.
```shell
rosrun basics topic_publisher.py
```
- 토픽을 실행한다.
```shell
rostopic list
```
```shell
/counter
/rosout
/rosout_agg
```
- counter 토픽이 광고되는 것을 검증할 수 있다.
```shell
rostopic echo counter -n 5
```
- -n 5 플래그를 통해 다섯 개의 메시지만 출력하도록 한다.
```shell
rostopic hz counter
```
- counter 토픽이 지정한 속도대로 발행하는지 검증하기 위해 사용할 수 있다.
```shell
rostopic info counter
```
```shell
Type: std_msgs/Int32

Publishers: 
 * /topic_publisher (http://localhost:37549/)

Subscribers: None
```
- Type, Publishers, Subscribers 등의 정보를 확인할 수 있다.
- counter 는 topic\_publisher에 의해 현재 Advertised, Subscribing Node는 없는 Int32 자료형 메시지를 운반하는 Topic 이다.
- topic\_publisher 발행자는 localhost 컴퓨터에서 실행되고 있고, TCP port 37549를 통해서 통신하고 있다.
## Topic Subscribing
```python
import rospy
from std_msgs.msg import Int32

def callback(msg):
    print(msg.data)

rospy.init_node("topic_subscriber")

sub = rospy.Subscriber("counter", Int32, callback)

rospy.spin()
```
- callback() : 메시지가 들어올 때 처리하는 callback 이다.
- ROS는 이벤트 구동 시스템으로 콜백 함수를 아주 많이 사용한다.
```python
sub = rospy.Subscriber("counter", Int32, callback)
```
- Topic 이름, 메시지 자료형, 콜백함수 이름을 지정한다.
- 내부적으로 Subscriber는 이 Info를 roscore로 전달해서 직접적으로 이 Topic의 Publisher와 연결하려고 한다.
- 토픽이 존재하지 않아도 **오류가 발생하지 않고** 해당 토픽이 발행되어 메시지 전송이 시작될 때까지 계속 ***기다린다***
```python
rospy.spin()
```
- Subscribing이 시작되면 ROS에게 제어를 넘겨준다.
- 이 함수는 노드가 종료될 준비가 될 경우에만 제어를 반환한다.
  - While 반복문 정의를 회피할 수 있는 유용한 방법이다.
  - ROS가 반드시 실행되는 main thread를 넘겨받을 필요가 없는 것이다.
- [참고1](https://gnaseel.tistory.com/31)
  - spin은 queue를 사용해서 먼저 요청된 콜백함수부터 처리한다. 또한, 노드가 정지되기 전까지 무한루프 처럼 동작하여 콜백함수를 끊임없이 처리한다.
- [참고2](https://bigbigpark.tistory.com/29)
  - ros에서 msg가 토픽으로 수신되면 그것을 queue에 쌓는다. 그 후 Subscriber의 spin 메소드를 통해서 queue에 있는 메세지를 처리하도록 구성되어있다.
- [참고3](https://stella47.tistory.com/111)
  - 새 메시지가 도착하면 ROS는 콜백 함수를 부를 수 있기 전까지 queue에 계속 저장해둔다고한다. 메시지 큐가 가득차면 큐의 가장 오래된 메시지를 지우고 넣는다.
  - 노드가 꺼지기 전까지 ROS에게 대기를 요청하고, 콜백을 실행할 권한을 요청한다.
- 종합해서 생각해보자면, spin은 queue에 쌓여있는 data를 처리할 **callback을 기억한채로 ROS에게 권한을 요청해서 승인 받을 시 이 callback을 호출**하는데, 노드가 죽을 때까지 **이 행위를 무한 반복**한다.
```shell
rosrun basics topic_subscriber.py
```
- counter 토픽으로 발행한 Int 정수가 출력되면 잘된 것이다.
```shell
rostopic info counter
```
```shell
Type: std_msgs/Int32

Publishers: 
 * /topic_publisher (http://192.168.123.7:43119/)

Subscribers: 
 * /topic_subscriber (http://192.168.123.7:44023/)
```
- 정보를 확인해서 검증할 수 있다.
## Latched Topic
- ROS 메세지를 순식간에 사라지므로 토픽 발행 시 바로 Subscribe하지 못하면 메시지를 잃어버리고 다음 메시지를 기다려야된다.
  - map을 구독하는 경우 다른 노드가 map 데이터가 필요한데 이 것이 오지않고 노드가 실행 되버릴 수 있는 것이다. (Real-Time 시 문제 발생)
  - 적절한 발행주기를 선택하는 것도 하나의 방법이지만 올바른 값을 구하는 것이 쉽지 않을 수 있다.
  - 이러한 문제를 해결하기 위한 것이 Latched Topic이다.
- Topic Advertise 시, Latch된다고 설정하면 Subscriber는 Topic을 Subscribe할 때 전송된 제일 **마지막 메시지**를 자동으로 얻게 된다.
  - 이는 Node가 Latched Topic을 발행함으로써 메시지를 한 번만 Publish하면 된다는 것을 뜻한다.
```python
pub = rospy.Publisher("map", nav_msgs/OccupancyGrid, latched=True)
```

