# roscore
- Node 들에게 "연결 정보"를 제공
  - 서로 메시지를 전송할 수 있도록 하는 서비스다.
- 모든 Node는 **roscore 로 연결**해서 Publish, Subscribe 하길 원하는 **Message Stream의 세부사항**을 등록
- roscore 없이는 Node가 another Node 를 찾을 수가 없다.
  - 무조건 실행시켜야한다.
- P2P 연결을 형성하는데 필요한 정보를 제공한다.
  - [peer to peer](https://shineover.tistory.com/258)
  - ROS의 핵심은 노드간 메시지가 P2P로 전송된다는 점이다. 이는 roscore는 **상대 노드가 어디있는지만 알려주기 위해 사용된다는 것을 의미**한다.
## How to serve linking between Nodes ?
- ROS Node 시작 시, **ROS\_MASTER\_URI**라는 **환경변수**를 찾기 시작한다.
```shell
hostname:11311
```
- 네트워크를 통해 접근 가능한 hostname이라는 호스트 상에서 11311 포트로 접근할 수 있는 roscore 실행 프로그램이 있다는 것을 뜻한다.
- Node가 네트워크 상 roscore의 위치를 알면 Data Stream을 찾기위해 "이름"을 사용해서 **질의한다.**
  - Publish 하려는 message info, subscribe 하고자하는 것에 대한 info
  - roscore는 관련된 Message 의 **생산자**, **소비자**의 **주소**를 알려준다.
- 모든 Node 들은 **주기적**으로 **상대 노드를 찾기위해** roscore가 제공하는 서비스를 호출한다.
## Parameter Server
- 매개변수 서버란, Node가 Robot에 대한 detailed info, algorithm에 대한 매개변수 등 과 같은 임의 자료 구조를 저장하고 불러올 수 있게 한다.
- ** 이 부분은 추후 공부가 필요한 것으로 보인다.**

