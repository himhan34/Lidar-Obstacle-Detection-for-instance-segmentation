// PCL을 사용하여 간단한 3D 고속도로 환경 생성
// 자율 주행 자동차 센서 탐색용

#include "sensors/lidar.h" // LiDAR 센서 관련 헤더 파일 포함
#include "render/render.h" // 렌더링 관련 헤더 파일 포함
#include "processPointClouds.h" // 포인트 클라우드 처리 헤더 파일 포함
// 템플릿 사용으로 인해 .cpp 파일을 포함하여 링커 문제 해결
#include "processPointClouds.cpp"

// 고속도로 환경 초기화 함수
// 매개변수:
// - renderScene: 환경을 렌더링할지 여부
// - viewer: PCL 시각화 객체
// 반환값:
// - 생성된 자동차 객체 리스트
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // 자차(egoCar)와 다른 자동차들(car1, car2, car3)을 초기화
  Car egoCar(Vect3(0,  0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar"); // 자차
  Car car1(  Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");  // 자동차 1
  Car car2(  Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");  // 자동차 2
  Car car3(  Vect3(-12,4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");  // 자동차 3

  // 자동차 객체를 벡터에 추가
  std::vector<Car> cars;
  cars.push_back(egoCar); // 자차 추가
  cars.push_back(car1);   // 자동차 1 추가
  cars.push_back(car2);   // 자동차 2 추가
  cars.push_back(car3);   // 자동차 3 추가

  // 렌더링 옵션이 활성화된 경우 환경 및 자동차 렌더링
  if (renderScene) {
    renderHighway(viewer); // 고속도로 렌더링
    egoCar.render(viewer); // 자차 렌더링
    car1.render(viewer);   // 자동차 1 렌더링
    car2.render(viewer);   // 자동차 2 렌더링
    car3.render(viewer);   // 자동차 3 렌더링
  }

  // 자동차 리스트 반환
  return cars;
}

/// @deprecated
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----3D 뷰어 열고 간단한 고속도로 환경 표시 -----
  // ----------------------------------------------------

  // 렌더링 옵션
  bool renderScene = false; // 고속도로 환경 렌더링 여부 설정
  std::vector<Car> cars = initHighway(renderScene, viewer); // 고속도로 초기화

  /// LiDAR 센서 생성
  Lidar* lidarSensor = new Lidar(cars, 0.0); // 자동차와 평지 경사를 기반으로 LiDAR 생성

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidarSensor->scan(); // LiDAR로 스캔하여 포인트 클라우드 생성
  //renderRays(viewer, lidarSensor->position, inputCloud); // 광선 렌더링 (비활성화됨)
  //renderPointCloud(viewer, inputCloud, "inputCloud", Color(1.0, 1.0, 0.0)); // 입력 클라우드 렌더링 (비활성화됨)

  /// 포인트 프로세서 생성
  ProcessPointClouds<pcl::PointXYZ> pointProcessor; // 포인트 클라우드 처리 객체 생성

  // 평면 세분화 수행
  // RANSAC 알고리즘을 사용하여 평면 분리
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
    pointProcessor.RansacPlane(inputCloud, 100, 0.2); // 최대 100번 반복, 거리 임계값 0.2

  // 평면 분리 결과를 렌더링
  renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1.0, 0, 0)); // 장애물 포인트 클라우드 렌더링 (빨간색)
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1.0, 0)); // 평면 포인트 클라우드 렌더링 (초록색)

  /// 클러스터링
  // 장애물 포인트 클라우드에서 클러스터링 수행
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(
    segmentCloud.first, 1.0, 3, 30); // 거리 임계값 1.0, 최소 3개 포인트, 최대 30개 포인트

  int clusterId = 0; // 클러스터 ID 초기화

  // 클러스터 색상 리스트 정의
  std::vector<Color> colors = {
    Color(1, 0, 0), // 빨강
    Color(1, 1, 0), // 노랑
    Color(0, 0, 1)  // 파랑
  };

  // 각 클러스터에 대해 포인트 렌더링 및 경계 상자 렌더링
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    // 클러스터의 포인트 수 출력
    std::cout << "cluster size ";
    pointProcessor.numPoints(cluster);

    // 클러스터 포인트 렌더링
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // 클러스터 주위에 경계 상자 렌더링
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId; // 다음 클러스터 ID로 증가
  }
}

/// @brief 3D 뷰어를 열고 도시 블록을 표시
/// @param viewer PCL 시각화 객체
/// @param pointProcessorI 포인트 클라우드 처리 객체
/// @param inputCloud 입력 포인트 클라우드
void cityBlock(
  pcl::visualization::PCLVisualizer::Ptr&   viewer,
  ProcessPointClouds<pcl::PointXYZI>* const pointProcessorI,
  pcl::PointCloud<pcl::PointXYZI>::Ptr&     inputCloud)
{
  // Step 0: PCD 파일 로드 (주석 처리됨)
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
  //  pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // Step 1: Voxel Grid 필터 적용
  inputCloud = pointProcessorI->FilterCloud(
    inputCloud,
    0.3, // 필터 크기 (0.3m)
    Eigen::Vector4f (-20, -6, -3, 1), // 최소 좌표 (x, y, z)
    Eigen::Vector4f ( 30,  7,  2, 1)); // 최대 좌표 (x, y, z)
  //renderPointCloud(viewer, inputCloud, "inputCloud"); // 필터링된 클라우드 렌더링 (비활성화)

  // Step 2: 필터링된 클라우드를 도로와 장애물로 분리
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->RansacPlane(inputCloud, 100, 0.2); // RANSAC 평면 분리

  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0)); // 도로 렌더링 (초록색)
  renderPointCloud(viewer, segmentCloud.first,  "obstCloud", Color(1, 0, 0)); // 장애물 렌더링 (빨간색)

  // Step 3: 장애물 클라우드 클러스터링
  KdTree* tree = new KdTree; // K-d 트리 생성
  for (int i = 0; i < segmentCloud.first->points.size(); ++i) {
    tree->insert(segmentCloud.first->points[i], i); // 장애물 포인트 삽입
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(
    segmentCloud.first, tree, 0.35, 15, 500); // 유클리드 클러스터링 (최소 15, 최대 500 포인트)

  int clusterId = 0; // 클러스터 ID 초기화

  // 클러스터 색상 정의
  std::vector<Color> colors = {
    Color(1, 0, 0), // 빨강
    Color(1, 1, 0), // 노랑
    Color(0, 0, 1)  // 파랑
  };

  // 각 클러스터를 렌더링
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    // 클러스터 포인트 수 출력
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);

    // 클러스터 포인트 렌더링
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // Step 4: 클러스터 주위에 경계 상자 렌더링
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId; // 다음 클러스터 ID로 증가
  }

  delete tree; // K-d 트리 메모리 해제
}

// setAngle: 카메라 각도 설정 {XY, TopDown, Side, FPS}
void initCamera(
  CameraAngle                             setAngle, // 카메라 각도 설정
  pcl::visualization::PCLVisualizer::Ptr& viewer) // PCL 시각화 객체
{
  viewer->setBackgroundColor(0, 0, 0); // 배경색을 검정색으로 설정

  // 카메라 위치와 각도 설정
  viewer->initCameraParameters();
  int distance = 16; // 카메라와의 거리 (단위: 미터)

  // 설정된 각도에 따라 카메라 위치를 변경
  switch(setAngle)
  {
    case XY: // XY 평면에서 카메라 위치
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown: // 위에서 아래로 내려다보는 위치
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side: // 측면에서 보는 위치
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS: // 1인칭 시점
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
      break;
  }

  // FPS 모드가 아닌 경우 좌표축을 추가
  if (setAngle != FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}

int main (int argc, char** argv)
{
  // 환경 시작 출력
  std::cout << "starting environment" << std::endl;

  // PCL 시각화 객체 생성
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY; // 초기 카메라 각도 설정
  initCamera(setAngle, viewer); // 카메라 초기화

  // 포인트 클라우드 처리 객체 생성
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

  // PCD 파일 스트림 읽기
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

  auto streamIterator = stream.begin(); // 스트림의 첫 번째 파일로 초기화
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  // 뷰어가 멈추지 않은 동안 반복
  while (!viewer->wasStopped()) {
    // 뷰어 초기화
    viewer->removeAllPointClouds(); // 모든 포인트 클라우드 제거
    viewer->removeAllShapes(); // 모든 도형 제거

    // PCD 파일 로드 및 장애물 탐지 프로세스 실행
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string()); // 현재 PCD 파일 로드
    cityBlock(viewer, pointProcessorI, inputCloudI); // 도시 블록 프로세스 실행

    // 다음 PCD 파일로 이동
    ++streamIterator;
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin(); // 스트림 끝에 도달하면 처음으로 돌아감
    }

    viewer->spinOnce(); // 한 프레임 업데이트
  }

  delete pointProcessorI; // 포인트 클라우드 처리 객체 메모리 해제

  return 0; // 프로그램 종료
}



