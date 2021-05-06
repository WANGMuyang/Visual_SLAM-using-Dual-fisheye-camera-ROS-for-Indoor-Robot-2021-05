# Visual_SLAM using Dual fisheye camera & ROS for Indoor Robot
전자공학 졸업작품 - 360 카메라(Dual Fisheye)로 Visual-SLAM & 목적지 설정 후 주행

1. OpenVSLAM의 내부 구조, Map Data 저장과 관련된 부분을 수정하여 목적지에 해당하는 Station 파트 추가.

2. 이를 Socket Viewer를 통해 나타내기 위해 /viewer/app.js 수정

3. 목적지 설정을 위한 Single_image_publisher.py
 - Map에 목적지를 설정해주는 Node를 동작시키고 Single_image_publisher.py를 실행하면 
 - 설정된 Frame에 해당하는 위치를 찾아서 Station으로 등록

4. openvslam_occ_grid-master를 통해 SLAM을 통해 생성된 Map msgpack에 대해 Localization 수행, 
   현재 카메라의 Orientation까지 파악

5. 목적지에 해당하는 Station을 향해 이동.

 <center><img src="https://user-images.githubusercontent.com/59793091/113572266-dcc03200-9652-11eb-966b-8fe90e12c87e.png" width="200" height="300"></center>

![image](https://user-images.githubusercontent.com/59793091/113572481-3d4f6f00-9653-11eb-82c5-280c6b6e477f.png)

 해당 이미지를 Equirectangular 형태로 변환한 후 SLAM 실행(Ricoh theta 배포 API를 이용한 변환)

![image](https://user-images.githubusercontent.com/59793091/113572497-450f1380-9653-11eb-98d1-d3f0454987db.png)

 실행 결과
 
 (Input image의 중심이 차량의 주행방향과 반대방향을 향하는 상태임에도 결과가 잘 나오는 모습이다.)
 
<img src="https://user-images.githubusercontent.com/59793091/113572861-fada6200-9653-11eb-8962-896e05c3b78e.gif" width="500" height="250">

 실행 결과 (전자공학부 복도 일부)

<img src="https://user-images.githubusercontent.com/59793091/113572946-23625c00-9654-11eb-8606-2407549ee595.png" width="500" height="250">

Localization은 실시간으로 진행해야 하므로 360 카메라의 이미지를 빠르게 

Equirectangular 형태로 변환할 수 있게 하기 위해 OpenGL을 이용하여 ROS Node 제작. 이를 통한 Localization 결과

Localization 테스트시 사용한 영상은 Input image의 중심이 차량의 주행방향과 같은 영상으로 결과가 잘 나오는 모습.

사용 node 링크 : https://github.com/sksguswjd717/Equirectangular-conversion-tools-for-Dual-fisheye-ROS

<img src="https://user-images.githubusercontent.com/59793091/113671455-527adb00-96f1-11eb-9095-af5333451186.gif" width="500" height="250">


