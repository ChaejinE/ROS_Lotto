# roslaunch
- 여러개의 ROS Node들의 시작을 자동화하도록 설계된 명령행 도구
- Node 보다는 오히려 launch file, 시작 파일을 통해 동작한다.
  - launch file : 여러 Nodes에 대한 토픽 Remapping과 매개변수를 기술한 XML File
```xml
<launch>
  <node name="talker" pkg="rospy_tutorials"
 	type="talker.py" output="screen" />
  <node name="listener" pkg="rospy_tutorials"
	type="listener.py" output="screen" />
</launch>
```
- \<node> : ROS graph **name**, 노드가 존재하는 **package**, 실행 프로그램의 파일이름 **type** 을 포함한다.
- output="screen" : log 파일 대신 console 출력해야 한다는 것을 가리키는 **속성**
  - **디버깅을 위해 보편적으로 사용**하는 설정이다.
```shell
roslaunch rospy_tutorials talker_listener.launch
```
- 모든 노드를 종료할 수 있으며 launch file을 실행한 창에서 Ctrl + c를 누르면된다.
- roslaunch는 호출되었을 때 **roscore가 존재하지 않으면** 자동으로 roscore를 생성한다.
  - 하지만, Ctrl + c 하면 **이 roscore는 종료된다.**
  - 그래도 이렇게 하면 전체 시스템을 연결해주는 roscore를 잃어버릴 위험이 없어진다.


