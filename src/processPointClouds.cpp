// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"

// 생성자
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// 소멸자
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

// 입력된 포인트 클라우드의 포인트 개수를 출력하는 함수
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // 클라우드 포인트 개수를 출력
  std::cout << cloud->points.size() << std::endl;
}

// 포인트 클라우드에서 보폭 필터링과 영역 필터링을 수행하는 함수
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
  typename pcl::PointCloud<PointT>::Ptr cloud, // 입력 포인트 클라우드
  float                                 filterRes, // 보폭 필터 크기
  Eigen::Vector4f                       minPoint, // 관심 영역 최소 좌표
  Eigen::Vector4f                       maxPoint) // 관심 영역 최대 좌표
{
  // 필터링 프로세스의 시작 시간 기록
  auto startTime = std::chrono::steady_clock::now();

  // 보폭 필터 객체 생성 및 필터 크기 설정
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud); // 입력 클라우드를 설정
  vg.setLeafSize(filterRes, filterRes, filterRes); // 필터 크기를 설정
  vg.filter(*cloudFiltered); // 필터 적용 후 결과를 저장

  // 관심 영역 필터링을 위한 클라우드 객체 생성
  typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

  // 관심 영역 설정 및 필터링
  pcl::CropBox<PointT> region(true); // 관심 영역 필터 객체 생성
  region.setMin(minPoint); // 최소 좌표 설정
  region.setMax(maxPoint); // 최대 좌표 설정
  region.setInputCloud(cloudFiltered); // 필터링할 클라우드를 설정
  region.filter(*cloudRegion); // 필터 적용 후 결과를 저장

  std::vector<int> indices;

  // 차량 지붕 영역에 있는 포인트를 필터링
  pcl::CropBox<PointT> roof(true); // 지붕 영역 필터 객체 생성
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1)); // 지붕 영역 최소 좌표 설정
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1)); // 지붕 영역 최대 좌표 설정
  roof.setInputCloud(cloudRegion); // 필터링할 클라우드를 설정
  roof.filter(indices); // 지붕 포인트 인덱스를 추출

  // 추출된 인덱스를 저장할 객체 생성
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  for (int point : indices) {
    // 각 인덱스를 inliers에 추가
    inliers->indices.push_back(point);
  }

  // 지붕 영역 포인트 제거
  pcl::ExtractIndices<PointT> extract; // 인덱스 기반 필터 객체 생성
  extract.setInputCloud(cloudRegion); // 필터링할 클라우드를 설정
  extract.setIndices(inliers); // 제거할 인덱스 설정
  extract.setNegative(true); // 선택된 포인트를 제외하고 필터링
  extract.filter(*cloudRegion); // 필터 적용 후 결과를 저장

  // 필터링 프로세스의 종료 시간 기록
  auto endTime = std::chrono::steady_clock::now();
  // 경과 시간 계산 및 출력
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  // 필터링 결과를 반환
  return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
  pcl::PointIndices::Ptr                inliers, // 평면에 속한 점들의 인덱스
  typename pcl::PointCloud<PointT>::Ptr cloud) // 입력된 포인트 클라우드
{
  // 두 개의 새로운 포인트 클라우드를 생성 (하나는 장애물, 다른 하나는 분할된 평면)
  typename pcl::PointCloud<PointT>::Ptr obstCloud {new pcl::PointCloud<PointT>()}; // 장애물 클라우드
  typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>()}; // 평면 클라우드

  // inliers 인덱스를 사용하여 평면에 속한 점들을 planeCloud에 추가
  for (int index : inliers->indices) {
    planeCloud->points.push_back(cloud->points[index]);
  }

  // 장애물 클라우드를 추출하기 위한 필터 객체 생성
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud); // 입력 클라우드 설정
  extract.setIndices(inliers); // inliers 설정
  extract.setNegative(true); // inliers가 아닌 포인트를 추출
  extract.filter(*obstCloud); // 필터 적용 후 결과를 obstCloud에 저장

  // 장애물 클라우드와 평면 클라우드를 결과로 반환
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

  return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
  typename pcl::PointCloud<PointT>::Ptr cloud, // 입력 포인트 클라우드
  int                                   maxIterations, // RANSAC 최대 반복 횟수
  float                                 distanceThreshold) // 거리 임계값
{
  // 분할 프로세스 시작 시간 기록
  auto startTime = std::chrono::steady_clock::now();

  // 클라우드에서 inliers를 찾기 위한 객체 생성
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr       inliers {new pcl::PointIndices}; // 평면에 속한 점들의 인덱스
  pcl::ModelCoefficients::Ptr  coefficients {new pcl::ModelCoefficients}; // 평면 모델 계수

  // 평면 모델 설정
  seg.setOptimizeCoefficients(true); // 계수 최적화 설정
  seg.setModelType(pcl::SACMODEL_PLANE); // 평면 모델로 설정
  seg.setMethodType(pcl::SAC_RANSAC); // RANSAC 방법 설정
  seg.setMaxIterations(maxIterations); // 최대 반복 횟수 설정
  seg.setDistanceThreshold(distanceThreshold); // 거리 임계값 설정

  // 입력 클라우드에서 가장 큰 평면 구성 요소 분할
  seg.setInputCloud(cloud); // 입력 클라우드 설정
  seg.segment(*inliers, *coefficients); // 평면 분할 수행

  // 평면 모델을 추정할 수 없는 경우 경고 출력
  if (inliers->indices.size() == 0) {
    std::cout << "Couldn't estimate a planar model for the given dataset." << std::endl;
  }

  // 분할된 클라우드들을 SeparateClouds 함수로 나눔
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

  // 분할 프로세스 종료 시간 기록
  auto endTime = std::chrono::steady_clock::now();
  // 경과 시간 계산 및 출력
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  // 분할 결과 반환
  return segResult;
}

template<typename PointT>
// 템플릿을 사용하여 PointT 타입의 포인트를 처리하는 함수 선언

std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  int                                   maxIterations,
  float                                 distanceTol)
{
  // 평면을 추출하기 위한 RANSAC 알고리즘 구현

  // 시간 측정을 시작합니다.
  auto startTime = std::chrono::steady_clock::now();

  // 최종적으로 가장 많은 inlier를 저장할 변수 초기화
  std::unordered_set<int> inliersResult;
  srand(time(nullptr)); // 난수 생성을 위한 시드 설정

  // 최대 반복 횟수 동안 실행
  while (maxIterations--) {
    std::unordered_set<int> inliers;

    // 평면을 정의하기 위해 임의로 3개의 점 선택
    while (inliers.size() < 3) {
      inliers.insert(rand() % (cloud->points.size())); // 포인트 클라우드에서 임의의 인덱스 선택
    }

    float x1, y1, z1; // 첫 번째 점의 좌표
    float x2, y2, z2; // 두 번째 점의 좌표
    float x3, y3, z3; // 세 번째 점의 좌표

    auto itr = inliers.begin(); // 선택된 inlier의 반복자 초기화

    // 첫 번째 점의 좌표 가져오기
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;
    // 두 번째 점의 좌표 가져오기
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;
    // 세 번째 점의 좌표 가져오기
    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;

    // 평면의 방정식 계수 계산
    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1); // 상수항 계산

    // 모든 점에 대해 반복
    for (int index = 0; index < cloud->points.size(); ++index) {
      if (inliers.count(index) > 0) {
        continue; // 이미 선택된 inlier는 스킵
      }

      PointT point = cloud->points[index]; // 현재 점 가져오기
      float x4 = point.x; // 점의 x 좌표
      float y4 = point.y; // 점의 y 좌표
      float z4 = point.z; // 점의 z 좌표

      // 평면과 점 사이의 거리 계산
      float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

      // 거리가 임계값보다 작으면 inlier로 추가
      if (dist <= distanceTol) {
        inliers.insert(index);
      }
    }

    // 현재 inlier가 기존보다 많으면 업데이트
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  // 시간 측정 종료 및 결과 출력
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Plane RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

  // inlier와 outlier 포인트 클라우드 생성
  typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

  // 각 포인트를 inlier와 outlier로 분리
  for (int index = 0; index < cloud->points.size(); ++index) {
    PointT point = cloud->points[index]; // 현재 점 가져오기

    if (inliersResult.count(index)) {
      cloudInliers->points.push_back(point); // inlier에 추가
    }
    else {
      cloudOutliers->points.push_back(point); // outlier에 추가
    }
  }

  // outlier와 inlier 클라우드를 반환
  return std::pair<typename pcl::PointCloud<PointT>::Ptr,
                   typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
}

template<typename PointT>
// 템플릿을 사용하여 PointT 타입의 포인트를 처리하는 클러스터링 함수 선언

std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  float                                 clusterTolerance,
  int                                   minSize,
  int                                   maxSize)
{
  // 클러스터링 처리 시간을 측정하기 위한 시작 시간 기록
  auto startTime = std::chrono::steady_clock::now();

  // 클러스터 결과를 저장할 벡터 선언
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // 클러스터링을 위한 k-d 트리 객체 생성
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud); // 입력 포인트 클라우드를 k-d 트리에 설정

  // 클러스터 인덱스를 저장할 벡터
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec; // 유클리드 클러스터링 객체 생성

  // 클러스터링 파라미터 설정
  ec.setClusterTolerance(clusterTolerance); // 클러스터링 거리 임계값
  ec.setMinClusterSize(minSize); // 최소 클러스터 크기
  ec.setMaxClusterSize(maxSize); // 최대 클러스터 크기
  ec.setSearchMethod(tree); // 검색 방법으로 k-d 트리 설정
  ec.setInputCloud(cloud); // 입력 포인트 클라우드 설정
  ec.extract(clusterIndices); // 클러스터링 수행 및 결과 저장

  // 클러스터링된 각 그룹에 대해 처리
  for (pcl::PointIndices getIndices : clusterIndices) {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

    // 클러스터에 속하는 포인트를 클러스터 포인트 클라우드에 추가
    for (int index : getIndices.indices) {
      cloudCluster->points.push_back(cloud->points[index]);
    }

    // 클러스터 포인트 클라우드의 메타데이터 설정
    cloudCluster->width = cloudCluster->points.size(); // 클러스터의 점 개수
    cloudCluster->height = 1; // 클러스터는 단일 레이어
    cloudCluster->is_dense = true; // 밀집 클러스터 설정

    clusters.push_back(cloudCluster); // 클러스터를 결과 벡터에 추가
  }

  // 클러스터링 처리 시간 측정 종료
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found "
            << clusters.size() << " clusters" << std::endl;

  return clusters; // 클러스터 결과 반환
}

template<typename PointT>
// 템플릿을 사용하여 재귀적으로 클러스터를 확장하는 도우미 함수 선언

void ProcessPointClouds<PointT>::clusterHelper(
  int                                   idx,
  typename pcl::PointCloud<PointT>::Ptr cloud,
  std::vector<int>&                     cluster,
  std::vector<bool>&                    processed,
  KdTree*                               tree,
  float                                 distanceTol) {

  processed[idx] = true; // 현재 인덱스를 처리 완료로 표시
  cluster.push_back(idx); // 클러스터에 현재 인덱스를 추가

  // 현재 포인트에서 거리 임계값 내에 있는 이웃 포인트 찾기
  std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);

  // 각 이웃 포인트에 대해 처리
  for (int id : nearest) {
    if (!processed[id]) { // 아직 처리되지 않은 경우
      // 재귀적으로 클러스터 확장
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
    }
  }
}

/// @brief 유클리드 클러스터링을 수행하여 각 클러스터의 인덱스를 반환하는 함수
/// @param[out] vector<vector<int>> - 각 클러스터에 해당하는 인덱스 리스트
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(
  typename pcl::PointCloud<PointT>::Ptr cloud, // 입력 포인트 클라우드
  KdTree*                               tree, // K-d 트리 포인터
  float                                 distanceTol, // 거리 임계값
  int                                   minSize, // 최소 클러스터 크기
  int                                   maxSize) // 최대 클러스터 크기
{
  // 클러스터를 저장할 벡터
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // 처리 여부를 확인하기 위한 벡터 초기화
  std::vector<bool> processed(cloud->points.size(), false);

  // 모든 포인트를 순회
  for (int idx = 0; idx < cloud->points.size(); ++idx) {
    if (!processed[idx]) { // 아직 처리되지 않은 경우
      std::vector<int> clusterIdx; // 현재 클러스터의 인덱스 저장
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>); // 클러스터 포인트 클라우드

      // 헬퍼 메서드 호출
      clusterHelper(idx, cloud, clusterIdx, processed, tree, distanceTol);

      // 클러스터 크기가 조건에 맞는 경우 추가
      if (clusterIdx.size() >= minSize && clusterIdx.size() <= maxSize) {
        for (int i = 0; i < clusterIdx.size(); ++i) {
          cluster->points.push_back(cloud->points[clusterIdx[i]]);
        }

        // 클러스터의 메타데이터 설정
        cluster->width = cluster->points.size();
        cluster->height = 1;

        clusters.push_back(cluster); // 클러스터 추가
      }
      else {
        // 조건을 만족하지 못하면 해당 포인트를 처리되지 않은 상태로 되돌림
        for (int i = 1; i < clusterIdx.size(); ++i) {
          processed[clusterIdx[i]] = false;
        }
      }
    }
  }

  return clusters; // 클러스터 결과 반환
}

/// @brief 클러스터의 바운딩 박스를 계산하는 함수
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
  typename pcl::PointCloud<PointT>::Ptr cluster) // 입력 클러스터
{
  PointT minPoint, maxPoint;

  // 클러스터의 최소, 최대 좌표 계산
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box; // 결과 바운딩 박스

  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box; // 바운딩 박스 반환
}

/// @brief 포인트 클라우드를 PCD 파일로 저장하는 함수
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(
  typename pcl::PointCloud<PointT>::Ptr cloud, // 저장할 포인트 클라우드
  std::string                           file) // 저장 경로
{
  // PCD 파일 ASCII 형식으로 저장
  pcl::io::savePCDFileASCII(file, *cloud);

  std::cerr << "Saved " << cloud->points.size() << " data points to " << file << std::endl;
}

/// @brief PCD 파일에서 포인트 클라우드를 로드하는 함수
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  // PCD 파일을 로드
  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
  }

  std::cerr << "Loaded " << cloud->points.size() << " data points from " << file << std::endl;

  return cloud; // 로드한 포인트 클라우드 반환
}

/// @brief 지정된 경로의 PCD 파일 목록을 반환하는 함수
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
  // 디렉토리 내의 모든 파일 경로를 가져와서 벡터로 저장
  std::vector<boost::filesystem::path> paths(
    boost::filesystem::directory_iterator{dataPath},
    boost::filesystem::directory_iterator{});

  // 파일 경로를 오름차순으로 정렬
  sort(paths.begin(), paths.end());

  return paths; // 정렬된 파일 경로 반환
}


