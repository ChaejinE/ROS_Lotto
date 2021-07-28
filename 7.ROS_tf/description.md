# tf: 좌표변환
- 널리 사용되는 관례가 있더라도 원점을 임의로 선택할 수 있다.
  - x-axis : 전방
  - y-axis : 왼쪽
  - z-axis : 위, 오른손 좌표계를 사용하면 z 방향은 추정이된다.
## Position
- (x, y, z) vector
- **원점을 기준**으로 ***얼마나 이동했는지***
## Orientation
- (r, p, y) vector
- **원점을 기준**으로 ***얼마나 회전했는지***
## Pose
- (x, y, z, r, p, y) 6D pose
- (위치, 방향) 쌍을 "자세"라고 한다.
##
- 로봇의 다른 요소 각각에 대한 Pose(자세) 를 계산할 수 있어야한다.
  - 알 필요가 있는 질문
    - 1. Base의 원점에 대한 자세에 대해서 Laser의 원점에 대한 자세는 무엇인가 ?
    - 2. Base에 대한 카메라의 자세
  - Static Relationship : ex) 이동체에 고정된 레이저
  - Dynamic Relationship : ex) 물건을 잡기 위해 뻗은 손
  - 센서 데이터와 액추에이터 명령으로 쉽게 변환할 수 있는 방식으로 결합할 필요가 있다.
- "변환 데이터"를 가지고 작업하는 모든 노드가 발행, 구독, 기억 또는 계산하는 것을 처음부터 만들 필요 없이 tf는 모든 노드에서 이러한 공통 작업을 수행하도록 사용할 수 있는 **라이브러리 집합을 제공**한다.
## 문서에 따르면...
- [tf pkackage 문서](http://wiki.ros.org/tf?distro=melodic)
- tf는 user가 시간에 따라 multiple 좌표 프레임들을 track하도록 하는 Package 라고 한다.
- tf는 시간에따라 쌓여진 tree 구조로 좌표 프레임들간 relationship을 유지한다고 한다.
- tf는 user가 시간에 따라 요구 되어지는 점에서 두 좌표 프레임들간 points, vector 등을 변환하도록 해준다고한다.
  - tf를 이용해서 두 프레임간의 변환행렬을 얻을 수 있다.
### 왜 Transform 하고 왜 tf를 사용하는가 ?
- robotic system은 일반적으로 시간에 따라 변화하는 3D 좌표 프레임들을 가지고 있다.
  - world frame, base frame, gripper frame, head frame, etc.
- tf는 시간에 따른 이러한 모든 프레임들을 추적하고, 아래와 같은 질문을 허용한다고 한다.
  - 5초 전에 World frame에대해 상대적으로 head frame은 어디에 있는가 ?
  - Base에 대해 상대적으로 gripper의 Pose는 어떠한가 ?
  - map frame에서 base frame의 현재 Pose는 어떠한가 ?
  - 이렇게 각 frame들 간의 관계를 알고, 얻기위해 tf를 사용한다고 이해하면 된다.
- tf는 **분산 시스템으로 작동**하기 때문에 모든 ROS component가 좌표 프레임에 대한 정보를 이용할 수 있다.
  - 변환 정보에 대한 중앙 server가 없다는 얘기가 된다.
### Tutorials
- tf를 사용하기 위한 두가지 필수적인 task들이 있다.
  - Listening for transforms
    - 시스템에서 broadcast된 좌표 프레임들을 받고, 쌓아 놓는다.
    - 프레임간에 특정 변환을 query 한다.
  - Broadcasting transforms
    - 좌표 프레임들에 대한 상대 pose를 받고자하는 다른 시스템들에게 보낸다.
    - 한 system은 많은 broadcaster를 가질수 있다. 이는 각각 다른 로봇 파트에 대한 정보를 제공할 수 있다.

