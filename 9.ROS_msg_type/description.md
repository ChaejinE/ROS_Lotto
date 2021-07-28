|자료형|C++|Python|비고|
|------|------|-------|
|bool|uint8\_t|bool||
|int8|int8\_t|int||
|uint8|uint8\_t|int|uint8\[]은 python:string취급|
|int16|uint16\_t|int||
|uint16|uint16\_t|int||
|int32|int32\_t|int||
|uint32|uint32\_t|int||
|int64|int64\_t|long||
|uint64|uint64\_t|long||
|float32|float|float||
|float64|double|float||
|string|std::string|string|UTF-8 encoding|
|time|ros::Time|rospy.Time|duration자료 참고|

# 사용자 메시지 자료형 정의
- [msg자료형 참고](http://wiki.ros.org/msg#Field_Types?distro=melodic)
- 자료형의 배열은 python의 튜플로 반환되며 튜플이나 리스트로 사용할 수 있다.
- C++은 기본적으로 더 많은 기본 자료형을 가지고 있어서 Node가 C++와 파이썬으로 작성되면 미묘한 문제가 발생할 수 있다.
  - uint8 일 때, C++에서는 정상 작동하겠지만, python에서는 정수로 표현되어 음수나 255보다 큰 값을 가지게 할 수 있다.
  - 수신되었을 때 예측하기 어려우므로 범위가 제한되는 ROS자료형 사용시 주의해야한다.

## 새 메시지 정의
- package의 msg directory에 있는 특수한 메시지 정의 파일을 사용해서 ROS 메시지를 정의한다.
- 이 파일은 코드에서 사용할 수 있는 언어별 구현으로 Compile 된다.
  - catkin\_make를 해야한다. 안그러면 새롭게 정의한 메시지 자료형을 찾을 수 없게된다.
  - catkin\_make로 언어별 코드를 생성하면 토픽 전송을 위한 자료형을 마샬링하고 언마샬링 하는 코드를 포함시킨다. 여기서 마샬링은 ROS 메시지 자료형을 네트워크 전송에 적합한 자료형으로 변환하는 과정이고, 언마샬링은 그 반대 개념이다. 덕분에 하나의 언어로 작성된 노드는 다른 언어로 작성된 노드로부터 토픽을 구독할 수 있게 된다.
```shell
vim Complex.msg
```
- Complex.msg를 msg 디렉토리 안에 생성한다.
- 물론, basics package 까지 모두 만들어졌다는 가정하에 이 패키지 안에서 msg 디렉토리가 있다는 말이다.
```
float32 real
float32 imaginary
```
- 위 내용을 적어주도록한다.
``` xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```
- package.xml 파일에 두 줄을 추가해준다.
- ROS가 언어별 메시지 코드를 생성할 수 있도록 **새로운 메시지 정의**를 **빌드 시스템에 알려주고자 하는 과정**이다.
```txt
find_package(
...
  roscpp
  rospy
  std_msgs
  message_generation
)
```
- CMakeLists.txt파일에서 다른 패키지들 다음에 message\_generation을 추가해주면 된다.
```txt
catkin_package(
  CATKIN_DEPENDS message_runtime
)
```
- CMakeLists.txt 파일에서 실행 시에 메시지를 사용한다고 catkin에게 알리는 과정이다.
```txt
add_message_files(
  FILES
  Complex.msg
)
```
- CMakeLists.txt 파일에서 호출에 메시지 파일을 추가하여 어떤 것을 컴파일할지 catkin에게 알리는 과정이다.
```txt
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
- CMakeLists.txt 파일에서 generat\_messages() 호출에 대한 주석처리를 제거하고 **사용자 메시지에 필요한 모든 종속성**을 포함시킬 필요가 있다.
- 이로써 사용자 메시지에 대해 알아야하는 **모든 것**을 catkin에게 알려주었다. catkin\_make 를 실행하면 된다.
- catkin\_make 실행 시 .msg 확장자가 제거된 형태로 메시지 정의 파일과 동일한 이름을 가지는 메시지 자료형을 생성한다. (관습상 ROS 자료형은 대문자 사용)

