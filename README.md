# 3DOF_manupulator_simulation

## Brief
이 프로젝트는 4자유도를 가진 매니퓰레이터를 라그랑지안 방정식을 이용한 동역학 해석을 통하여 시뮬레이션한 프로젝트입니다.  
서울과학기술대학교의 응용 로봇 설계 수업의 텀 프로젝트로 제작되었습니다.
시뮬레이트 툴박스는 Peter Corke의 [RTB](https://petercorke.com/toolboxes/robotics-toolbox/)를 사용하였습니다.
||||
|---|---|---|
|IDE|MATLAB| R2020b|
</br>

## Tree
||||
|---|---|---|
|app_temproj.m|최종 완성된 프로젝트 코드입니다.|
|dh_parameter.m|간단히 나타낸 DH-Parameter를 쉽게 구할 수 있는 코드입니다.|
|homeework.m| 3-DOF 로봇의 라그랑지안 방정식을 이용한 동역학 해석입니다.|
|report1.m| homework.m과 똑같지만 벡터 변수로 만든 코드입니다.|
|verify_FF_IK.m|역기구학으로 구한 각도값을 정기구학을 이용해 검산하는 코드입니다.|