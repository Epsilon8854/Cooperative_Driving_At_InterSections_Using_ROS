# Cooperative_Driving_At_InterSections_Using_V2I
## Overview
---
이 패키지는 V2I(car <-> traffic light)와 카메라 인식 기술을 활용하여 교차로 도로에서 충돌없는 주행을 가능케합니다.

![](https://imgur.com/UM0WUk2.jpg)
### CASE1 | 신호 위반 차량 : 빨간색
---
![](https://github.com/Epsilon8854/Cooperative_Driving_At_InterSections_Using_ROS/blob/main/images/case1.gif?raw=true)
### CASE2 | 신호 위반 차량 : 빨간색
---
![](https://github.com/Epsilon8854/Cooperative_Driving_At_InterSections_Using_ROS/blob/main/images/case2.gif?raw=true)

## Localization
---
사진과 같이 차량에 Aruco marker를 부착하여 신호등(카메라)기준 차량의 위치를 인식할 수 있겠금 개발하였습니다.

- aruco marker 부착 모습
  
![](https://imgur.com/6rNKbV0.jpg)

- 위치인식 결과
![](https://github.com/Epsilon8854/Cooperative_Driving_At_InterSections_Using_ROS/blob/main/images/arucoDetect.gif)