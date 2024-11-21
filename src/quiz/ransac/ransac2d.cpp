// RANSAC을 사용한 간단한 선형 피팅 구현 퀴즈

#include "../../render/render.h" // 렌더링 관련 헤더 파일 포함
#include <unordered_set>         // 유일한 요소를 저장하는 컨테이너 사용
#include "../../processPointClouds.h" // 포인트 클라우드 처리 유틸리티
#include "../../processPointClouds.cpp" // 템플릿 관련 .cpp 파일 포함

// 2D 데이터 생성 함수
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // 인라이어 추가
  float scatter = 0.6; // 점의 흩어진 정도 설정

  for(int i = -5; i < 5; i++) // -5부터 5까지 점 생성
  {
    double rx = 2*(((double) rand() / (RAND_MAX))-0.5); // x축 임의 값 생성
    double ry = 2*(((double) rand() / (RAND_MAX))-0.5); // y축 임의 값 생성

    pcl::PointXYZ point; // 포인트 생성
    point.x = i + scatter * rx; // x 좌표
    point.y = i + scatter * ry; // y 좌표
    point.z = 0;                // z 좌표는 0으로 설정

    cloud->points.push_back(point); // 클라우드에 점 추가
  }

  // 아웃라이어 추가
  int numOutliers = 10; // 아웃라이어 개수 설정

  while(numOutliers--) // 아웃라이어가 0이 될 때까지 반복
  {
    double rx = 2*(((double) rand() / (RAND_MAX))-0.5); // x축 임의 값 생성
    double ry = 2*(((double) rand() / (RAND_MAX))-0.5); // y축 임의 값 생성

    pcl::PointXYZ point; // 포인트 생성
    point.x = 5 * rx;    // x 좌표
    point.y = 5 * ry;    // y 좌표
    point.z = 0;         // z 좌표는 0으로 설정

    cloud->points.push_back(point); // 클라우드에 점 추가
  }

  cloud->width  = cloud->points.size(); // 클라우드 너비 설정
  cloud->height = 1;                    // 클라우드 높이 설정

  return cloud; // 생성된 클라우드 반환
}

// 3D 데이터 생성 함수
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
  ProcessPointClouds<pcl::PointXYZ> pointProcessor; // 포인트 클라우드 처리기 생성
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd"); // PCD 파일 로드
}

// 시각화 초기화 함수
pcl::visualization::PCLVisualizer::Ptr initScene()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));

  viewer->setBackgroundColor (0, 0, 0); // 배경색 설정
  viewer->initCameraParameters();      // 카메라 파라미터 초기화
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0); // 카메라 위치 설정
  viewer->addCoordinateSystem (1.0);   // 좌표계 추가

  return viewer; // 초기화된 뷰어 반환
}

// 2D RANSAC 알고리즘
std::unordered_set<int> Ransac(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, // 포인트 클라우드
  int                                 maxIterations, // 최대 반복 횟수
  float                               distanceTol)   // 거리 허용 오차
{
  // 알고리즘 실행 시간 측정 시작
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult; // 인라이어 결과 저장
  srand(time(NULL)); // 난수 초기화

  while (maxIterations--) { // 최대 반복 횟수만큼 반복
    std::unordered_set<int> inliers; // 현재 반복에서의 인라이어 저장

    while (inliers.size() < 2) { // 랜덤으로 2개의 점 선택
      inliers.insert(rand() % (cloud->points.size()));
    }

    auto itr = inliers.begin(); // 선택된 점의 좌표 가져오기
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;

    float a = (y1 - y2); // 직선 방정식의 기울기 계산
    float b = (x2 - x1); // 직선 방정식의 y 절편 계산
    float c = (x1*y2 - x2*y1); // 직선 방정식의 상수 계산

    for (int index = 0; index < cloud->points.size(); ++index) { // 모든 점에 대해 반복
      if (inliers.count(index) > 0) {
        continue; // 이미 선택된 점은 건너뜀
      }

      pcl::PointXYZ point = cloud->points[index]; // 현재 점 가져오기
      float x3 = point.x;
      float y3 = point.y;

      // 점과 직선 사이의 거리 계산
      float d = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);

      if (d <= distanceTol) { // 거리가 허용 오차 이내이면 인라이어로 추가
        inliers.insert(index);
      }
    }

    if (inliers.size() > inliersResult.size()) { // 가장 많은 인라이어를 가진 결과 저장
      inliersResult = inliers;
    }
  }

  // 알고리즘 실행 시간 측정 종료
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

  return inliersResult; // 결과 반환
}

// 메인 함수
int main ()
{
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(); // 시각화 초기화

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); // 3D 데이터 생성

  std::unordered_set<int> inliers = RansacPlane(cloud, 10, 1.0); // RANSAC 실행

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];

    if (inliers.count(index)) {
      cloudInliers->points.push_back(point); // 인라이어 점 저장
    }
    else {
      cloudOutliers->points.push_back(point); // 아웃라이어 점 저장
    }
  }

  if (inliers.size()) { // 인라이어와 아웃라이어 렌더링
    renderPointCloud(viewer, cloudInliers,  "inliers",  Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  }
  else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped ()) { // 뷰어 루프
    viewer->spinOnce ();
  }
}
