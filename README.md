# Equirectangular conversion tool for Dual fisheye & ROS

## Overview
Ricoh Theta S를 사용하며 Live로 기존의 이미지에서 Equirectanguler로 변환하기 위해 opengl_ros 를 사용

 - 이미지 변환 공식을 GLSL을 통해 구현
 - OpenVSLAM과 ROS를 사용한 indoor robot을 위해 제작한 node

```bash
rosrun opengl_ros defisheye
```
