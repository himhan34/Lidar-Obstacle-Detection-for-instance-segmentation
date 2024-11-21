// RANSAC 기반 간단한 선형 피팅 구현을 연습하는 퀴즈

#include "../../render/render.h" // 렌더링 관련 헤더 파일 포함
#include "../../render/box.h"   // 박스 렌더링 관련 헤더 파일 포함
#include <chrono>               // 시간 측정용 헤더 파일 포함
#include <string>               // 문자열 관련 헤더 파일 포함
#include "kdtree.h"             // KD 트리 관련 헤더 파일 포함

// 인자 설명:
// window는 박스를 그릴 영역을 정의
// zoom은 카메라 확대 수준을 조정
pcl::visualization::PCLVisualizer::Ptr initScene(
  Box window, // 박스 영역 정보
  int zoom    // 카메라 확대 비율
)
{
  // PCL 시각화 뷰어 생성
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));

  viewer->setBackgroundColor (0, 0, 0); // 배경색을 검정색으로 설정
  viewer->initCameraParameters();       // 카메라 파라미터 초기화
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0); // 카메라 위치 설정
  viewer->addCoordinateSystem (1.0);              // 좌표계 추가

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window"); // 박스 추가

  return viewer; // 생성된 뷰어 반환
}

// 포인트 데이터를 생성
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>()); // 포인트 클라우드 객체 생성

  for (int i = 0; i < points.size(); i++) // 각 포인트 데이터를 반복
  {
    pcl::PointXYZ point;        // 포인트 객체 생성
    point.x = points[i][0];     // x 좌표 설정
    point.y = points[i][1];     // y 좌표 설정
    point.z = 0;                // z 좌표는 0으로 설정

    cloud->points.push_back(point); // 클라우드에 포인트 추가
  }

  cloud->width = cloud->points.size(); // 클라우드의 너비 설정
  cloud->height = 1;                   // 클라우드의 높이는 1로 설정

  return cloud; // 생성된 클라우드 반환
}

// KD 트리를 2D로 렌더링
void render2DTree(
  Node*                                   node,     // 현재 노드
  pcl::visualization::PCLVisualizer::Ptr& viewer,  // PCL 뷰어
  Box                                     window,   // 현재 노드의 영역
  int&                                    iteration,// 반복 횟수
  uint                                    depth=0   // 현재 깊이
)
{
  if (node != nullptr) { // 노드가 비어있지 않을 경우
    Box upperWindow = window; // 상단 영역
    Box lowerWindow = window; // 하단 영역

    // x축 기준으로 나눌 경우
    if (depth % 2 == 0) {
      viewer->addLine(
        pcl::PointXYZ(node->point[0], window.y_min, 0),
        pcl::PointXYZ(node->point[0], window.y_max, 0),
        0, 0, 1, "line" + std::to_string(iteration)); // 수직선 추가

      lowerWindow.x_max = node->point[0]; // 하단 영역 x축 최대값 수정
      upperWindow.x_min = node->point[0]; // 상단 영역 x축 최소값 수정
    }
    // y축 기준으로 나눌 경우
    else {
      viewer->addLine(
        pcl::PointXYZ(window.x_min, node->point[1], 0),
        pcl::PointXYZ(window.x_max, node->point[1], 0),
        1, 0, 0, "line" + std::to_string(iteration)); // 수평선 추가

      lowerWindow.y_max = node->point[1]; // 하단 영역 y축 최대값 수정
      upperWindow.y_min = node->point[1]; // 상단 영역 y축 최소값 수정
    }

    ++iteration; // 반복 횟수 증가

    // 재귀적으로 왼쪽과 오른쪽 서브트리 처리
    render2DTree(node->left,  viewer, lowerWindow, iteration, depth + 1);
    render2DTree(node->right, viewer, upperWindow, iteration, depth + 1);
  }
}

// 클러스터를 생성하기 위한 도우미 함수
void clusterHelper(
  int                                    idx,       // 현재 포인트 인덱스
  const std::vector<std::vector<float>>& points,    // 전체 포인트 데이터
  std::vector<int>&                      cluster,   // 현재 클러스터
  std::vector<bool>&                     processed, // 처리 여부 기록
  KdTree*                                tree,      // KD 트리
  float                                  distanceTol// 거리 허용 범위
)
{
  processed[idx] = true; // 현재 포인트를 처리 완료로 표시
  cluster.push_back(idx);// 현재 포인트를 클러스터에 추가

  std::vector<int> nearest = tree->search(points[idx], distanceTol); // 거리 내 가까운 포인트 검색

  for (int id : nearest) { // 검색된 포인트 중 처리되지 않은 포인트를 재귀적으로 처리
    if (!processed[id]) {
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
    }
  }
}

// 유클리드 클러스터링을 통해 클러스터 생성
std::vector<std::vector<int>> euclideanCluster(
  const std::vector<std::vector<float>>& points, // 포인트 데이터
  KdTree*                                tree,   // KD 트리
  float                                  distanceTol // 거리 허용 범위
)
{
  std::vector<std::vector<int>> clusters;         // 클러스터 리스트
  std::vector<bool> processed(points.size(), false); // 처리 여부 기록

  int i = 0;
  while (i < points.size()) { // 모든 포인트 처리
    if (processed[i]) {       // 이미 처리된 포인트는 건너뜀
      ++i;
      continue;
    }

    std::vector<int> cluster; // 새로운 클러스터 생성
    clusterHelper(i, points, cluster, processed, tree, distanceTol); // 클러스터 도우미 호출
    clusters.push_back(cluster); // 클러스터 추가
    ++i;
  }

  return clusters; // 생성된 클러스터 반환
}

int main ()
{
  // 시각화 뷰어 생성

  Box window; // 박스 객체 초기화
  window.x_min = -10; // x축 최소값
  window.x_max =  10; // x축 최대값
  window.y_min = -10; // y축 최소값
  window.y_max =  10; // y축 최대값
  window.z_min =   0; // z축 최소값
  window.z_max =   0; // z축 최대값

  // 초기화된 박스를 사용하여 뷰어 생성
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

  // 데이터 생성
  std::vector<std::vector<float>> points = {
    {-6.2,  7},   {-6.3, 8.4}, {-5.2,  7.1}, // 포인트 데이터
    {-5.7,  6.3}, {7.2,  6.1}, { 8.0,  5.3},
    { 7.2,  7.1}, {0.2, -7.1}, { 1.7, -6.9},
    {-1.2, -7.2}, {2.2, -8.9}
  };
  // 포인트 데이터를 사용하여 포인트 클라우드 생성
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

  // KD 트리 생성
  KdTree* tree = new KdTree;
  
    for (int i = 0; i < points.size(); i++) { 
      tree->insert(points[i],i); // KD 트리에 포인트 삽입
    }

    int it = 0;
    // KD 트리를 2D로 렌더링
    render2DTree(tree->root, viewer, window, it);

    // KD 트리 검색 테스트
    std::cout << "Test Search" << std::endl;
    std::vector<int> nearby = tree->search({-6,7},3.0); // 반경 3.0 내 검색

    for (int index : nearby) { 
      std::cout << index << ","; // 검색 결과 출력
    }
    std::cout << std::endl;

    // 클러스터링 처리 시간 측정 시작
    auto startTime = std::chrono::steady_clock::now();
    //
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0); // 유클리드 클러스터링
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // 클러스터링 결과 출력
    std::cout << "clustering found " << clusters.size()
              << " and took " << elapsedTime.count()
              << " milliseconds" << std::endl;

    // 클러스터 렌더링
    int clusterId = 0;
    std::vector<Color> colors = {
      Color(1, 0, 0), // 빨간색
      Color(0, 1, 0), // 초록색
      Color(0, 0, 1)  // 파란색
    };

    for (std::vector<int> cluster : clusters)
    {
      // 각 클러스터에 대해 포인트 클라우드 생성
      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());

      for (int indice : cluster) {
        clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0], points[indice][1], 0));
      }

      // 클러스터를 뷰어에 렌더링
      renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
      ++clusterId; // 클러스터 ID 증가
    }

    if (clusters.size() == 0) { 
      // 클러스터가 없을 경우 원본 데이터를 렌더링
      renderPointCloud(viewer, cloud, "data");
    }

    // 뷰어가 종료될 때까지 루프 실행
    while (!viewer->wasStopped()) {
      viewer->spinOnce (); // 한 프레임씩 렌더링
    }
}

