#ifndef LIDAR_H
#define LIDAR_H

#include "../render/render.h" // 렌더링 관련 헤더 파일 포함
#include <ctime>              // 시간 관련 라이브러리 포함
#include <chrono>             // 시간 측정 관련 라이브러리 포함

// 원주율 상수 정의
const double pi = 3.1415;

// 광선 구조체 정의
struct Ray
{
  Vect3  origin;       // 광선의 시작 위치
  double resolution;   // 광선의 단계 크기 (정확도 및 계산 비용 결정)
  Vect3  direction;    // 광선의 방향 벡터
  Vect3  castPosition; // 현재 광선이 도달한 위치
  double castDistance; // 광선이 이동한 거리

  // 생성자: 광선의 속성 초기화
  // 매개변수:
  // - setOrigin: 광선의 시작 위치
  // - horizontalAngle: xy 평면에서 광선의 방향 각도
  // - verticalAngle: xy 평면과 광선 간의 방향 각도
  // - setResolution: 광선 단계 크기 (작을수록 정확하지만 더 많은 계산 필요)
  Ray(
    Vect3  setOrigin,
    double horizontalAngle,
    double verticalAngle,
    double setResolution)
    : origin(setOrigin)
    , resolution(setResolution)
    , direction(resolution * cos(verticalAngle) * cos(horizontalAngle), // x 방향 계산
                resolution * cos(verticalAngle) * sin(horizontalAngle), // y 방향 계산
                resolution * sin(verticalAngle)) // z 방향 계산
    , castPosition(origin) // 초기 위치를 시작 위치로 설정
    , castDistance(0) // 초기 이동 거리를 0으로 설정
  {}

  // 광선 캐스팅 함수
  // 매개변수:
  // - cars: 충돌 감지할 자동차 리스트
  // - minDistance: 광선의 최소 유효 거리
  // - maxDistance: 광선의 최대 유효 거리
  // - cloud: 포인트 클라우드 데이터 저장 객체
  // - slopeAngle: 경사면의 각도
  // - sderr: 표준 편차를 사용한 노이즈 추가 값
  void rayCast(
    const std::vector<Car>&              cars, // 자동차 목록
    double                               minDistance, // 최소 유효 거리
    double                               maxDistance, // 최대 유효 거리
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, // 포인트 클라우드 객체
    double                               slopeAngle, // 경사면의 각도
    double                               sderr) // 표준 편차 노이즈 값
  {
    // 광선 초기화
    castPosition = origin; // 시작 위치로 초기화
    castDistance = 0; // 이동 거리 초기화

    bool collision = false; // 충돌 여부 플래그

    // 충돌이 없고 최대 거리 내에서 반복
    while (!collision && castDistance < maxDistance) {
      castPosition = castPosition + direction; // 방향 벡터만큼 위치 이동
      castDistance += resolution; // 이동 거리 증가

      // 경사면과의 충돌 여부 확인
      collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

      // 자동차와의 충돌 여부 확인
      if (!collision && castDistance < maxDistance) {
        for (Car car : cars) {
          collision |= car.checkCollision(castPosition); // 충돌 여부 업데이트
          if (collision) {
            break; // 충돌 시 반복 종료
          }
        }
      }
    }

    // 유효 거리 범위 내에 있는 경우 포인트 추가
    if ((castDistance >= minDistance) &&
        (castDistance <= maxDistance))
    {
      // 표준 편차를 사용하여 노이즈 추가
      double rx = ((double) rand() / (RAND_MAX)); // x축 노이즈
      double ry = ((double) rand() / (RAND_MAX)); // y축 노이즈
      double rz = ((double) rand() / (RAND_MAX)); // z축 노이즈

      // 포인트 클라우드에 포인트 추가
      cloud->points.push_back(
        pcl::PointXYZ(castPosition.x + rx * sderr, // x 좌표에 노이즈 추가
                      castPosition.y + ry * sderr, // y 좌표에 노이즈 추가
                      castPosition.z + rz * sderr)); // z 좌표에 노이즈 추가
    }
  }
};

struct Lidar
{
  std::vector<Ray>                    rays; // 광선의 리스트
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // 포인트 클라우드 데이터
  std::vector<Car>                    cars; // 자동차의 리스트
  Vect3                               position; // LiDAR의 위치

  double groundSlope; // 지면의 기울기
  double minDistance; // LiDAR의 최소 감지 거리
  double maxDistance; // LiDAR의 최대 감지 거리
  double resoultion;  // 광선의 단계 크기 (해상도)
  double sderr;       // 표준 편차로 인한 노이즈 값

  // 생성자: LiDAR의 속성 초기화
  // 매개변수:
  // - setCars: 자동차 리스트
  // - setGroundSlope: 지면의 기울기
  Lidar(
    std::vector<Car> setCars,
    double setGroundSlope)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>()), // 포인트 클라우드 초기화
      position(0, 0, 2.6) // LiDAR의 초기 위치 설정
  {
    // 최소 거리를 5m로 설정하여 자차 루프의 점 제거
    minDistance = 5;
    maxDistance = 50;
    resoultion  = 0.2;

    // 표준 편차를 0.2로 설정하여 더 흥미로운 pcd 파일 생성
    sderr       = 0.2;
    cars        = setCars;
    groundSlope = setGroundSlope;

    // 레이어 수를 8로 늘려 더 높은 해상도의 pcd 생성
    int numLayers = 8;

    // 가장 가파른 수직 각도 설정
    double steepestAngle = 30.0 * (-pi/180); // 라디안으로 변환
    double angleRange    = 26.0 * (pi/180);  // 수직 각도 범위

    // 수평 각도 증가 값을 pi/64로 설정하여 더 높은 해상도의 pcd 생성
    double horizontalAngleInc = pi/64;

    double angleIncrement = angleRange/numLayers; // 레이어 간 수직 각도 증가 값

    // 각 수직 각도에 대해 수평 방향으로 광선을 생성
    for (double angleVertical = steepestAngle;
         angleVertical < steepestAngle + angleRange;
         angleVertical += angleIncrement)
    {
      for (double angle = 0; angle <= 2 * pi; angle += horizontalAngleInc)
      {
        Ray ray(position, angle, angleVertical, resoultion); // 광선 생성
        rays.push_back(ray); // 광선 리스트에 추가
      }
    }
  }

  // 소멸자: 스마트 포인터로 메모리 관리를 자동화하므로 추가 작업 필요 없음
  ~Lidar()
  {}

  // LiDAR로 스캔하여 포인트 클라우드 생성
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan()
  {
    cloud->points.clear(); // 이전 포인트 클라우드 데이터를 초기화

    auto startTime = std::chrono::steady_clock::now(); // 시작 시간 기록

    // 각 광선에 대해 rayCast 호출
    for (Ray ray : rays) {
      ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
    }

    auto endTime = std::chrono::steady_clock::now(); // 종료 시간 기록
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // ray casting 시간 출력
    cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;

    // 포인트 클라우드 크기 설정
    cloud->width  = cloud->points.size(); // 포인트 개수
    cloud->height = 1; // 1차원 비조직화 포인트 클라우드

    return cloud; // 생성된 포인트 클라우드 반환
  }
};

#endif // LIDAR_H







