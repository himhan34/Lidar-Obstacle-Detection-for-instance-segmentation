#ifndef BOX_H
#define BOX_H

#include <Eigen/Geometry> 
// Eigen 라이브러리의 Geometry 모듈을 포함합니다. (벡터와 회전에 관련된 기능 제공)

// BoxQ 구조체 정의
struct BoxQ
{
  Eigen::Vector3f    bboxTransform; // 박스의 변환 정보 (3D 공간에서 위치 정보)
  Eigen::Quaternionf bboxQuaternion; // 박스의 회전 정보 (쿼터니언으로 표현)
  float              cube_length; // 박스의 길이
  float              cube_width; // 박스의 너비
  float              cube_height; // 박스의 높이
};

// Box 구조체 정의
struct Box
{
  float x_min; // 박스의 최소 x 좌표
  float y_min; // 박스의 최소 y 좌표
  float z_min; // 박스의 최소 z 좌표
  float x_max; // 박스의 최대 x 좌표
  float y_max; // 박스의 최대 y 좌표
  float z_max; // 박스의 최대 z 좌표
};

#endif
// BOX_H가 정의되지 않은 경우에만 이 파일의 내용을 정의하도록 합니다.
