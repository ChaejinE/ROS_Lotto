# Catkin일
- ROS가 실행 프로그램, 라이브러리, 스크립트 및 다른코드가 사용할 **Interface를 생성하는 데 사용하는 도구 집합**인***ROS 빌드 시스템***
- [참고1](http://wiki.ros.org/catkin?distro=melodic)
- [참고2](http://wiki.ros.org/catkin?conceptual_overview?distro=melodic)
- CMake 매크로들과 일반적인 CMake 작업 흐름에 추가적인 기능을 제공하기 위한 전용 파이썬 스크립트로 구성
## 작업 공간
- 관련된 ROS Code들이 있는 Directory 집합이다.
- **현재 작업 공간에 있는 코드만 볼 수 있다.**
```shell
source /opt/ros/distro/setup.bash
```
- 시스템 범위의 ROS 설정을 반드시 해야한다.
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- 보통 .bashrc에 설정을 지정해 놓는다.
```shell
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src
catkin_init_workspace
```
- catkin\_ws : 작업 공간(work space) directory와 그 안에 **Code를 위한** src directory를 생성한다.
- catkin\_init\_workspace : 명령을 호출한 src 디렉터리 안에 CMakeList.txt 파일을 생성
  - CMake File은 시스템 범위의 CMake File 에 대한 Symbolic link를 생성하는 것이라고 한다.
```shell
cd ./catkin_ws
catkin_make
```
- catkin\_make : devel, build 2개의 directory가 생성된다.
  - build : C++ 사용 시 **라이브러리와 실행 프로그램과 같은 catkin 작업 결과 중 일부를 저장**하는 곳이다. **파이썬을 사용하는 경우 대부분 무시된다.**
  - devel : 가장 중요한 환경설정 파일이 있다. 이를 실행할 시 **현재 작업 공간과 그 안에 포함된 코드를 사용하도록 설정**한다.
```shell
source devel/setup.bash
```
- 이를 마무리로 작업공간(WorkSpace)가 만들어졌으며, **모든 코드와 추가적으로 작성한 코드**를 이 공간 안 ROS 패키지로 구성된 **src 디렉터리에 두어야한다.**
- 새로운 shell(or terminal)을 열면 작업하고 싶은 작업 공간에 대한 setup.bash 파일을 source 명령으로 실행해야한다.
  - 보통 .bashrc에 source ~/catkin/ws/devel.setup.bash 명령을 추가한다.
  - 위 행위를 통해 자동으로 작업 공간을 설정할 수 있게 된다.
## ROS Package
- Package들은 작업 공간 내부 src directory에 둔다.
- 각 Package Directroy는  CMakeLists.txt 파일과 package.xml 파일을 포함해야한다.
  - package.xml File : Package 내용과 Catkin이 어떻게 연동되어야하는지 설명하는 파일
```shell
cd ./catkin_ws/src
catkin_create_pkg basics rospy
cd ./catkin_ws/src/basics
mkdir scripts && cd scripts
vim topic_publisher.py
vim topic_subscriber.py
cd ~./catkin_ws && catkin_make
rosrun basics topic_publisher.py
rosrun basics topic_subscriber.py
cd ./catkin_ws/src
```
- catkin\_create\_pkg : rospy 패키지에 의존하는 basics 라는 새로운 패키지를 만든다.
  - 새로운 패키지가 다른 기존 패키지에 의존하면 명령행에서 나열이 가능
  - 의존성에 대한 얘기는 나중에 의논된다.
  - 새로운 패키지와 같은 이름의 Directory, 그 안에 CMakeLists.txt, package.xml 파일, src directory를 함께 생성한다.
  - package.xml 파일은 새로운 패키지에 관한 Metadata 들을 포함한다.
```xml
<?xml version="1.0"?>
<package format="2">
  <name>basics</name>
  <version>0.0.0</version>
  <description>The basics package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="cjlotto@todo.todo">cjlotto</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/basics</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```
- package name 은 변경하지 말아야한다.
- \<version> : 버전 번호
- \<description> : 이 package 안에 무엇이 있으며 무엇을 위한 것이에 대한 짧은 설명
- \<maintainer> : package를 관리하고, 버그를 고치는 데 책임이 있는 사람은 누구인가 ?
- \<license> : 코드를 배포할 때 적용하는 라이선스는 ?
- \<url> : 패키지에 관한 ROS 위키 페이지를 가리키는 URL
- \<buildtool\_depend> : **buildtool pkg**에 대해서 사용, 패키지는 어떤 의존성이 있는지 ? -> 추후에 공부
  - \<build\_depend> : **컴파일 시간**에 필요한 패키지에 대해서 사용
  - \<test\_depend> : 테스트할 때만 필요한 패키지에 대해서 사용
- \<export> : catkin이 아닌 다른 도구가 사용하는 정보를 위한 부분
- exec\_depend, build\_export\_depend 는 추가되었나 보다..
##
- 패키지를 만들었으면 Python Node 및 다른 파일들을 src 디렉터리에 넣으면 된다.
- launch 파일은 보통 launch 디렉토리에 둔다 -> 곧 공부
