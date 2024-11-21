// 자동차와 고속도로와 같은 환경을 렌더링하기 위해 사용하는 함수와 구조체

#ifndef RENDER_H
#define RENDER_H

#include <pcl/visualization/pcl_visualizer.h> // PCL 시각화 관련 헤더 파일 포함
#include "box.h" // 박스 관련 구조체 정의 포함
#include <iostream> // 표준 입출력 사용을 위한 헤더 파일
#include <vector> // 벡터 자료구조 사용을 위한 헤더 파일
#include <string> // 문자열 처리를 위한 헤더 파일

// 색상을 나타내는 구조체 정의
struct Color
{
  float r; // 빨간색 값
  float g; // 초록색 값
  float b; // 파란색 값

  // 색상 초기화를 위한 생성자
  Color(
    float setR, // 빨간색 값 설정
    float setG, // 초록색 값 설정
    float setB) // 파란색 값 설정
    : r(setR) // 멤버 변수 r에 값 할당
    , g(setG) // 멤버 변수 g에 값 할당
    , b(setB) // 멤버 변수 b에 값 할당
  {}
};

// 3D 벡터를 나타내는 구조체 정의
struct Vect3
{
  double x; // x 좌표
  double y; // y 좌표
  double z; // z 좌표

  // 벡터 초기화를 위한 생성자
  Vect3(
    double setX, // x 좌표 값 설정
    double setY, // y 좌표 값 설정
    double setZ) // z 좌표 값 설정
    : x(setX) // 멤버 변수 x에 값 할당
    , y(setY) // 멤버 변수 y에 값 할당
    , z(setZ) // 멤버 변수 z에 값 할당
  {}

  // 벡터 덧셈 연산자 오버로딩
  Vect3 operator+(const Vect3& vec)
  {
    Vect3 result(
      x + vec.x, // x 좌표 값 더하기
      y + vec.y, // y 좌표 값 더하기
      z + vec.z); // z 좌표 값 더하기

    return result; // 덧셈 결과 반환
  }
};

// 카메라 시점 각도를 정의하는 열거형
enum CameraAngle
{
  XY,       // XY 평면 시점
  TopDown,  // 위에서 내려다보는 시점
  Side,     // 측면 시점
  FPS       // 1인칭 시점
};


struct Car
{
  // 자동차의 위치를 미터 단위로 정의
  Vect3 position; 

  // 자동차의 크기를 미터 단위로 정의
  Vect3 dimensions; 

  // 자동차 이름을 문자열로 정의
  std::string name; 

  // 자동차 색상을 정의
  Color color; 

  // 생성자: 자동차의 위치, 크기, 색상, 이름 초기화
  Car(
    Vect3 setPosition,
    Vect3 setDimensions,
    Color setColor,
    std::string setName)
    : position(setPosition)
    , dimensions(setDimensions)
    , color(setColor)
    , name(setName)
  {}

  // 자동차를 시각화하는 함수
  void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
  {
    // 자동차 하단 부분을 시각화
    viewer->addCube(
      position.x - dimensions.x/2, position.x + dimensions.x/2, // x축 범위 설정
      position.y - dimensions.y/2, position.y + dimensions.y/2, // y축 범위 설정
      position.z, position.z + dimensions.z*2/3, // z축 범위 설정
      color.r, color.g, color.b, // 색상 설정
      name); // 객체 이름 설정

    // 하단 부분의 렌더링 속성을 표면으로 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
      name);

    // 하단 부분의 색상 속성 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      color.r, color.g, color.b,
      name);

    // 하단 부분의 투명도를 1.0 (불투명)으로 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);

    // 자동차 상단 부분을 시각화
    viewer->addCube(
      position.x - dimensions.x/4, position.x + dimensions.x/4, // x축 범위 설정
      position.y - dimensions.y/2, position.y + dimensions.y/2, // y축 범위 설정
      position.z + dimensions.z*2/3, position.z + dimensions.z, // z축 범위 설정
      color.r, color.g, color.b, // 색상 설정
      name + "Top"); // 상단 객체 이름 설정

    // 상단 부분의 렌더링 속성을 표면으로 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
      name + "Top");

    // 상단 부분의 색상 속성 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      color.r, color.g, color.b,
      name + "Top");

    // 상단 부분의 투명도를 1.0 (불투명)으로 설정
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name + "Top");
  }

  // 두 값이 범위 내에 있는지 확인하는 보조 함수
  bool inbetween(
    double point, // 점의 좌표 값
    double center, // 중심 좌표 값
    double range) // 범위 값
  {
    return (center - range <= point) && // 좌표가 범위 시작보다 크거나 같음
           (center + range >= point); // 좌표가 범위 끝보다 작거나 같음
  }

  // 주어진 점이 자동차와 충돌했는지 확인
  bool checkCollision(Vect3 point)
  {
    return (inbetween(point.x, position.x, dimensions.x/2) && // 하단 충돌 조건 확인 (x축)
            inbetween(point.y, position.y, dimensions.y/2) && // 하단 충돌 조건 확인 (y축)
            inbetween(point.z, position.z + dimensions.z/3, dimensions.z/3)) || // 하단 충돌 조건 확인 (z축)
           (inbetween(point.x, position.x, dimensions.x/4) && // 상단 충돌 조건 확인 (x축)
            inbetween(point.y, position.y, dimensions.y/2) && // 상단 충돌 조건 확인 (y축)
            inbetween(point.z, position.z + dimensions.z*5/6, dimensions.z/6)); // 상단 충돌 조건 확인 (z축)
  }
};

// 고속도로를 시각화하는 함수
void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);

// 광선을 시각화하는 함수
void renderRays(
  pcl::visualization::PCLVisualizer::Ptr&    viewer, // 뷰어 객체
  const Vect3&                               origin, // 광선의 시작점
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud); // 대상 포인트 클라우드

// 광선을 제거하는 함수
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);

// 포인트 클라우드를 시각화하는 함수
void renderPointCloud(
  pcl::visualization::PCLVisualizer::Ptr&    viewer, // 뷰어 객체
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, // 대상 포인트 클라우드
  std::string                                name, // 이름
  Color                                      color = Color(1, 1, 1)); // 색상 (기본값: 흰색)

// 포인트 클라우드를 시각화하는 함수
void renderPointCloud(
  pcl::visualization::PCLVisualizer::Ptr&     viewer, // 뷰어 객체
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, // 대상 포인트 클라우드 (강도 값 포함)
  std::string                                 name, // 포인트 클라우드 이름
  Color                                       color = Color(-1, -1, -1)); // 색상 (기본값: 자동 할당)

// 3D 박스를 시각화하는 함수
void renderBox(
  pcl::visualization::PCLVisualizer::Ptr& viewer, // 뷰어 객체
  Box                                     box, // 축에 정렬된 박스 데이터
  int                                     id, // 박스의 고유 ID
  Color                                   color = Color(1, 0, 0), // 색상 (기본값: 빨강)
  float                                   opacity = 1); // 투명도 (기본값: 1, 완전 불투명)

// 회전된 3D 박스를 시각화하는 함수
void renderBox(
  pcl::visualization::PCLVisualizer::Ptr& viewer, // 뷰어 객체
  BoxQ                                    box, // 축에 비정렬된 (회전된) 박스 데이터
  int                                     id, // 박스의 고유 ID
  Color                                   color = Color(1, 0, 0), // 색상 (기본값: 빨강)
  float                                   opacity = 1); // 투명도 (기본값: 1, 완전 불투명)

#endif // RENDER_H
