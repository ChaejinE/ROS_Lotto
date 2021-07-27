#Name
- Node, Message Stream(Topic), Parameter는 유일한 이름을 가져야한다. (ROS에서)
## Name Space
- ROS는 이름 충돌을 피하고자 별도로 구분된 이름공간으로 **같은 노드들을 시작시킬 수 있다.**
  - ex) left/image, right/image 라는 이미지 스트림을 만들어 낼 수 있다.
  - 이것은 Topic Name 충돌을 피하게 만든다.
  - 하지만, image 토픽을 수신하길 기대하는 다른 프로그램들은 같은 이름 공간에서 실행되어야한다.
  - 따라서 재사상(Remapping)을 한다.
## Remapping
- ROS 설계 패턴은 소프트웨어의 재사용을 독려하므로 remapping name은 ROS 개발 및 배포에 매우 일반적으로 쓰인다.
- ROS는 Name Remapping 표준 문법을 제공한다.
```shell
./image_view image:=right/image
```
- ./image\_view : Node
- image:=right/image : image가 original topic name, right/image가 remapping topic name 이다.
```shell
./camera __ns:=right
```
- Node를 이름공간으로 넣으려면 특별한 \_\_ns 이름공간 재사상 문법으로 가능하다.
- 작업 디렉터리가 camera프로그램을 포함하고 있으면, 위의 셸 명령은 right 이름공간과 함께 camera를 실행한다.
- 위의 내용을 바탕으로 생각을 정리해보면, image\_view Node는 right라는 재사상된 이름공간을 가지는 camera Node에서 image topic (right/image)을 받을 수 있게 되는 것이다.
```shell
sh run_talker1.sh # rosrun rospy_tutorials talker __name:=talker1
sh run_talker2.sh # rosrun rospy_tutorials talker __name:=talker2
sh run_listener.sh # rosrun rospy_tutorials listener
sh run_rqt.sh # rqt_graph
```
- 노드에 새로운 이름을 재사상하고 싶다면 위와 같이 한다.
