// KD 트리를 구현하는 퀴즈

#include "../../render/render.h" // 렌더링 관련 헤더 파일 포함

// KD 트리의 노드를 표현하는 구조체
struct Node
{
  std::vector<float> point; // 2D 포인트
  int   id;                 // 포인트의 고유 ID
  Node* left;               // 왼쪽 자식 노드
  Node* right;              // 오른쪽 자식 노드

  // 노드 생성자
  Node(
    std::vector<float> arr, // 포인트 데이터
    int                setId // 포인트 ID
  )
  : point (arr) // 포인트 데이터 초기화
  , id    (setId) // ID 초기화
  , left  (nullptr) // 왼쪽 자식 초기화
  , right (nullptr) // 오른쪽 자식 초기화
  {}
};

// KD 트리를 표현하는 구조체
struct KdTree
{
  Node* root; // 트리의 루트 노드

  // KD 트리 생성자
  KdTree()
  : root(nullptr) // 루트 노드 초기화
  {}

  // 트리에 노드를 삽입하는 헬퍼 함수
  void insertHelper(
    Node**             node, // 현재 노드 포인터
    uint               depth, // 트리 깊이
    std::vector<float> point, // 삽입할 포인트
    int                id // 포인트 ID
  )
  {
    // 트리가 비어있는 경우
    if (*node == nullptr) {
      *node = new Node(point, id); // 새로운 노드 생성
    }
    else {
      // 현재 차원을 계산
      uint cd = depth % 2;

      // 삽입할 포인트의 좌표와 비교하여 왼쪽 또는 오른쪽으로 이동
      if (point[cd] < ((*node)->point[cd])) {
        insertHelper(&((*node)->left), depth + 1, point, id); // 왼쪽 자식으로 이동
      }
      else {
        insertHelper(&((*node)->right), depth + 1, point, id); // 오른쪽 자식으로 이동
      }
    }
  }

  // 트리에 포인트를 삽입
  void insert(
    std::vector<float> point, // 삽입할 2D 포인트
    int                id // 포인트의 고유 ID
  )
  {
    // 이 함수는 새로운 포인트를 트리에 삽입
    // 루트 노드에 올바르게 배치되도록 노드 생성 및 삽입
    insertHelper(&root, 0, point, id);
  }

  // 트리에서 타겟 포인트와의 거리 조건을 만족하는 노드를 검색하는 헬퍼 함수
  void searchHelper(
    const std::vector<float>& target, // 검색 대상 포인트
    Node*                     node, // 현재 노드
    const int                 depth, // 현재 트리 깊이
    const float&              distanceTol, // 거리 허용 오차
    std::vector<int>&         ids // 검색 결과로 반환될 ID 목록
  )
  {
    if (node != nullptr) { // 현재 노드가 존재할 경우
      // 현재 노드가 타겟의 허용 범위 안에 있는지 확인
      if ((node->point[0] >= target[0] - distanceTol && // x축 조건 확인
           node->point[0] <= target[0] + distanceTol) &&
          (node->point[1] >= target[1] - distanceTol && // y축 조건 확인
           node->point[1] <= target[1] + distanceTol)) {
        float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                              (node->point[1] - target[1]) * (node->point[1] - target[1])); // 유클리드 거리 계산

        if (distance <= distanceTol) { // 거리 조건을 만족할 경우
          ids.push_back(node->id); // 노드 ID를 결과 목록에 추가
        }
      }

      // 경계 조건 확인 후 재귀적으로 왼쪽/오른쪽 서브트리 탐색
      if (target[depth % 2] - distanceTol < node->point[depth % 2]) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }
      if (target[depth % 2] + distanceTol > node->point[depth % 2]) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
      }
    }
  }

  // 주어진 타겟 포인트와 허용 거리 내에 있는 포인트 ID를 반환
  std::vector<int> search(
    std::vector<float> target, // 검색 대상 포인트
    float              distanceTol // 거리 허용 오차
  )
  {
    std::vector<int> ids; // 검색 결과를 저장할 벡터
    searchHelper(target, root, 0, distanceTol, ids); // 헬퍼 함수 호출

    return ids; // 검색 결과 반환
  }
};
