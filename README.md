## 1. Project

![git overview.png](resources/overview.png)

**엣지 기반 자율주행 기능의 FallbackMRC에 따른 운영권 SW 안전성 및 대응 방안 검증 기술 개발**

**KETI-OSI-gRPC 인터페이스**는 자율주행 시뮬레이터(MORAI 등)와 자율주행 인지/판단/제어 알고리즘 간 표준화된 연동을 지원하는 인터페이스입니다.

이 프로젝트는 ROS 기반의 자율주행 오픈 플랫폼(Autoware, Apollo)과의 연동을 통해 센서 데이터, 차량 상태 정보, 제어 명령 등을 실시간으로 교환할 수 있도록 gRPC와 OSI 표준을 기반으로 인터페이스를 개발하였습니다.

## 2. Feature

이 인터페이스는 “자율주행기술개발혁신사업”의 일환으로 개발되었으며, 엣지 기반 자율주행 기능의 Fall back MRC 운영권 SW 안전성 및 대응 방안을 검증하기 위해 고안되었으며, 주요 특징은 다음과 같습니다.

- **표준 인터페이스**: [ASAM OSI(Open Simulation Interface)](https://github.com/OpenSimulationInterface/open-simulation-interface) 표준 규격에 따른 SensorView, GroundTruth, HostVehicleData 등 다양한 데이터 포맷 지원
- **다양한 언어 지원**: C++, C#, Python 등 여러 프로그래밍 언어 기반으로 구현 가능
- **오픈 플랫폼 연동**: [Autoware](https://github.com/autowarefoundation/autoware), [Apollo](https://github.com/ApolloAuto/apollo) 등 자율주행 오픈 플랫폼과 원활하게 연동되어 시뮬레이션 및 실차 검증에 활용 가능

## 3. Installation & Usage

### 필수 종속성 설치

- **ROS Noetic (Ubuntu 20.04)**
    
    설치 및 환경설정은 다음과 같이 진행합니다:
    
    ```bash
    bashsudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    ```
    
- **rosbridge-server**
    
    ```bash
    bashsudo apt-get update
    sudo apt-get install ros-noetic-rosbridge-server
    ```
    
- **protobuf 23.1, Eigen 3.3.9, gRPC 1.56.2, pybind11-dev, dotnet**
    
    각 라이브러리는 [매뉴얼 PDF](https://github.com/youngbo-shim/keti-osi-grpc/blob/main/doc/FallbackMRC_%EC%9E%90%EC%9C%A8%EC%A3%BC%ED%96%89%20SW%20%EA%B2%80%EC%A6%9D%EC%9A%A9%20%EC%9D%B8%ED%84%B0%ED%8E%98%EC%9D%B4%EC%8A%A4%20%EC%82%AC%EC%9A%A9%20%EB%A7%A4%EB%89%B4%EC%96%BC.pdf)에 명시된 bash 스크립트를 참고하여 설치하시기 바랍니다.
    

### 저장소 클론 및 빌드

1. 저장소를 클론하고 서브모듈을 업데이트합니다:
    
    ```bash
    git clone --recursive https://github.com/youngbo-shim/keti-osi-grpc.git
    cd osi-grpc
    ```
    
2. OSI 서브모듈 내의 CMakeLists.txt에서 C++ 표준을 14로 수정합니다:
    
    ```bash
    cd src/async_stream/open-simulation-interface
    sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 14)/g' CMakeLists.txt
    ```
    
3. 빌드를 실행합니다(ROS catkin workspace 환경에서):
    
    ```bash
    cd <your osi-grpc repository path>
    catkin_make
    ```
    
4. ROS 환경을 설정한 후, 다음 명령으로 인터페이스를 실행합니다:
    
    ```bash
    source devel/setup.bash
    roslaunch async_stream osi_grpc.launch
    ```
    
    이때, 필요에 따라 gRPC Server의 IP와 포트 번호는 launch 파일에서 수정해야 합니다. 자세한 설명을 [매뉴얼 PDF](https://github.com/youngbo-shim/keti-osi-grpc/blob/main/doc/FallbackMRC_%EC%9E%90%EC%9C%A8%EC%A3%BC%ED%96%89%20SW%20%EA%B2%80%EC%A6%9D%EC%9A%A9%20%EC%9D%B8%ED%84%B0%ED%8E%98%EC%9D%B4%EC%8A%A4%20%EC%82%AC%EC%9A%A9%20%EB%A7%A4%EB%89%B4%EC%96%BC.pdf)를 참고하시기 바랍니다.
    

## 6. **License**

이 프로젝트는 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 하에 배포됩니다. 자세한 내용은 [LICENSE](https://github.com/youngbo-shim/keti-osi-grpc/blob/main/LICENSE) 파일을 참고하세요.
