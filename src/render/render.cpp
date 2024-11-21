// 환경을 렌더링하기 위해 사용하는 함수와 구조체
// 자동차 및 고속도로와 같은 요소를 렌더링합니다.

#include "render.h" 
// 렌더링 기능을 제공하는 헤더 파일 포함

// 고속도로를 렌더링하는 함수
void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // 단위: 미터
  double roadLength = 50.0; // 도로의 길이
  double roadWidth  = 12.0; // 도로의 너비
  double roadHeight = 0.2;  // 도로의 높이

  // 도로 큐브 추가
  viewer->addCube(
    -roadLength / 2, roadLength / 2, // 도로의 x 좌표 범위
    -roadWidth  / 2, roadWidth  / 2, // 도로의 y 좌표 범위
    -roadHeight, 0,                  // 도로의 z 좌표 범위
    0.2, 0.2, 0.2,                   // 도로 색상 (회색)
    "highwayPavement");              // 도로의 이름

  // 도로 큐브의 렌더링 속성을 설정 (표면으로 표시)
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
    "highwayPavement");

  // 도로 큐브의 색상 속성 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    0.2, 0.2, 0.2, "highwayPavement");

  // 도로 큐브의 투명도 속성 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY,
    1.0, "highwayPavement");

  // 도로 차선 추가 (첫 번째 차선)
  viewer->addLine(
    pcl::PointXYZ(-roadLength / 2, -roadWidth / 6, 0.01), // 시작점
    pcl::PointXYZ( roadLength / 2, -roadWidth / 6, 0.01), // 끝점
    1, 1, 0,                                             // 색상 (노란색)
    "line1");                                            // 차선 이름

  // 도로 차선 추가 (두 번째 차선)
  viewer->addLine(
    pcl::PointXYZ(-roadLength / 2, roadWidth / 6, 0.01),  // 시작점
    pcl::PointXYZ( roadLength / 2, roadWidth / 6, 0.01),  // 끝점
    1, 1, 0,                                              // 색상 (노란색)
    "line2");                                             // 차선 이름
}

// 광선의 개수를 카운트하는 변수
int countRays = 0;

// 광선을 렌더링하는 함수
void renderRays(
  pcl::visualization::PCLVisualizer::Ptr&    viewer, // 렌더링 뷰어
  const Vect3&                               origin, // 광선의 시작점
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)  // 광선의 끝점이 포함된 점 구름
{
  // 점 구름의 각 점에 대해 반복
  for (pcl::PointXYZ point : cloud->points) {
    // 광선 추가 (시작점에서 끝점까지)
    viewer->addLine(
      pcl::PointXYZ(origin.x, origin.y, origin.z), // 시작점
      point,                                      // 끝점
      1, 0, 0,                                    // 색상 (빨간색)
      "ray" + std::to_string(countRays));         // 광선 이름

    countRays++; // 광선 개수 증가
  }
}

// 렌더링된 광선을 제거하는 함수
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // 모든 광선 제거
  while (countRays) {
    countRays--; // 광선 개수 감소
    viewer->removeShape("ray" + std::to_string(countRays)); // 광선 제거
  }
}

// 점 구름을 렌더링하는 함수
void renderPointCloud(
  pcl::visualization::PCLVisualizer::Ptr&    viewer, // 렌더링 뷰어
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  // 점 구름 데이터
  std::string                                name,   // 점 구름 이름
  Color                                      color)  // 점 구름 색상
{
  // 점 구름 추가
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);

  // 점 구름의 렌더링 속성 설정 (점 크기)
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);

  // 점 구름의 색상 속성 설정
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    color.r, color.g, color.b, // RGB 색상
    name);
}

// 포인트 클라우드를 렌더링하는 함수
void renderPointCloud(
  pcl::visualization::PCLVisualizer::Ptr&     viewer, // 렌더링 뷰어
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,  // 입력 포인트 클라우드
  std::string                                 name,   // 포인트 클라우드의 이름
  Color                                       color)  // 렌더링 색상
{
  if (color.r == -1) { 
    // 색상이 -1이면 포인트 클라우드의 강도 값에 기반해 색상을 선택
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");

    // 강도 기반 색상으로 포인트 클라우드 추가
    viewer->addPointCloud<pcl::PointXYZI>(
      cloud, intensity_distribution, name);
  }
  else {
    // 사용자가 입력한 색상을 기반으로 색상을 설정
    viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      color.r, color.g, color.b,
      name);
  }

  // 포인트 클라우드의 크기 속성 설정
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// 와이어프레임 박스를 투명 색상으로 채워 렌더링하는 함수
void renderBox(
  pcl::visualization::PCLVisualizer::Ptr& viewer, // 렌더링 뷰어
  Box                                     box,    // 박스의 좌표 정보
  int                                     id,     // 박스의 고유 ID
  Color                                   color,  // 박스의 색상
  float                                   opacity) // 박스의 투명도
{
  if (opacity > 1.0) {
    opacity = 1.0; // 최대 투명도를 1로 제한
  }

  if (opacity < 0.0) {
    opacity = 0.0; // 최소 투명도를 0으로 제한
  }

  std::string cube = "box" + std::to_string(id); // 박스의 이름 설정

  // 와이어프레임 박스 추가
  viewer->addCube(
    box.x_min, box.x_max, // x 좌표 범위
    box.y_min, box.y_max, // y 좌표 범위
    box.z_min, box.z_max, // z 좌표 범위
    color.r, color.g, color.b, // 색상
    cube);

  // 와이어프레임 속성 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
    cube);

  // 와이어프레임 색상 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    color.r, color.g, color.b,
    cube);

  // 와이어프레임 투명도 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY,
    opacity,
    cube);

  std::string cubeFill = "boxFill" + std::to_string(id); // 채운 박스 이름 설정

  // 채운 박스 추가
  viewer->addCube(
    box.x_min, box.x_max,
    box.y_min, box.y_max,
    box.z_min, box.z_max,
    color.r, color.g, color.b,
    cubeFill);

  // 채운 박스 속성을 표면으로 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
    cubeFill);

  // 채운 박스 색상 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    color.r, color.g, color.b,
    cubeFill);

  // 채운 박스 투명도 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY,
    opacity * 0.3, // 투명도를 30%로 줄임
    cubeFill);
}

// 변환 및 회전을 고려한 박스를 렌더링하는 함수
void renderBox(
  pcl::visualization::PCLVisualizer::Ptr& viewer, // 렌더링 뷰어
  BoxQ                                    box,    // 박스의 변환 및 회전 정보
  int                                     id,     // 박스의 고유 ID
  Color                                   color,  // 박스의 색상
  float                                   opacity) // 박스의 투명도
{
  if (opacity > 1.0) {
    opacity = 1.0; // 최대 투명도를 1로 제한
  }

  if(opacity < 0.0) {
    opacity = 0.0; // 최소 투명도를 0으로 제한
  }

  std::string cube = "box" + std::to_string(id); // 박스의 이름 설정

  // 변환과 회전을 적용한 박스 추가
  viewer->addCube(
    box.bboxTransform, // 박스의 변환 정보
    box.bboxQuaternion, // 박스의 회전 정보
    box.cube_length, // 박스의 길이
    box.cube_width,  // 박스의 너비
    box.cube_height, // 박스의 높이
    cube);

  // 와이어프레임 속성 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
    cube);

  // 와이어프레임 색상 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    color.r, color.g, color.b,
    cube);

  // 와이어프레임 투명도 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY,
    opacity,
    cube);

  std::string cubeFill = "boxFill" + std::to_string(id); // 채운 박스 이름 설정

  // 변환과 회전을 적용한 채운 박스 추가
  viewer->addCube(
    box.bboxTransform,
    box.bboxQuaternion,
    box.cube_length,
    box.cube_width,
    box.cube_height,
    cubeFill);

  // 채운 박스 속성을 표면으로 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
    cubeFill);

  // 채운 박스 색상 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    color.r, color.g, color.b,
    cubeFill);

  // 채운 박스 투명도 설정
  viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY,
    opacity * 0.3, // 투명도를 30%로 줄임
    cubeFill);
}


