using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualNavigator : MonoBehaviour
{
    [Header("Refs")]
    public StateData _state;
    public VirtualDeviceController _deviceController;

    [Header("옵션")]
    public bool forceStartGoalWalkable = true; // 시작/목표를 항상 Walkable로 보정
    public bool useTransformFallback = true; // _state.WorldPosition이 유효하지 않으면 transform.position 사용
    public float retryDelay = 0.5f; // 실패 시 재시도 간격(초)
    public int maxStartSearchRing = 25;   // 시작점 보정 시 최대 링(셀)
    public int maxGoalSearchRing = 25;   // 목표점 보정 시 최대 링(셀)

    private Node _start;
    private Node _end;

    void Start()
    {
        if (_deviceController == null)
            _deviceController = GetComponent<VirtualDeviceController>();

        if (_state == null)
            _state = GetComponent<StateData>();

        AssignTask();
    }

    /// <summary> 상태에 따라 목적지를 선택하고 경로 요청을 시작. </summary>
    public void AssignTask()
    {
        _end = GetEndPosition();
        if (_end == null)
        {
            Debug.LogWarning("[VirtualNavigator] 목적지 노드가 null입니다. 상태/맵 설정 확인 후 재시도합니다.");
            StartCoroutine(RetryAssignLater());
            return;
        }

        RequestAndMove();
    }

    /// <summary> AMR 상태에 따른 목적지 노드 결정. </summary>
    private Node GetEndPosition()
    {
        // Managers.Map.GetRandomLoaderPos() / GetRandomDroperPos() 가 Node를 반환한다고 가정.
        if (_state == null)
        {
            // 상태정보가 없으면 기본 목적지(Loader)로 보냄. 프로젝트 정책에 맞게 수정 가능.
            return Managers.Map.GetRandomLoaderPos();
        }

        switch (_state.AmrState)
        {
            case AMR_STATE.RUNNING:
            case AMR_STATE.DROP:
                return Managers.Map.GetRandomLoaderPos();
            case AMR_STATE.LOAD:
                return Managers.Map.GetRandomDroperPos();
            default:
                Debug.LogWarning($"[VirtualNavigator] 예외 상태: {_state.AmrState} → 기본 목적지(Loader) 사용");
                return Managers.Map.GetRandomLoaderPos();
        }
    }

    /// <summary> 시작/목표 노드를 안전하게 보정하고 Path를 요청한 뒤 주행 시작. </summary>
    private void RequestAndMove()
    {
        // 1) 시작 노드 안전 획득
        Vector3 worldStart = Vector3.zero;

        if (_state != null && _state.WorldPosition != Vector3.zero)
            worldStart = _state.WorldPosition;
        else if (useTransformFallback)
            worldStart = transform.position;
        else
            worldStart = transform.position;

        if (!TryGetNearestWalkable(worldStart, out _start, maxStartSearchRing))
        {
            Debug.LogError($"[VirtualNavigator] 시작 노드를 찾지 못했습니다. worldStart={worldStart}");
            StartCoroutine(RetryAssignLater());
            return;
        }

        // 2) 목적지 노드 보정 (null 또는 비워커블이면 근처 Walkable로 스냅)
        if (_end == null || !_end.Walkable)
        {
            // _end가 null일 수도 있으니, 가능한 한 월드좌표로 변환 후 근처 Walkable 탐색
            Vector3 goalWorld = (_end != null)
                ? CoordinateCalc.GridToWorld(_end.Pos, Managers.Map.Height, Managers.Map.Resolution)
                : transform.position; // fallback (실제로는 이 케이스 거의 없음)

            if (!TryGetNearestWalkable(goalWorld, out _end, maxGoalSearchRing))
            {
                Debug.LogError("[VirtualNavigator] 목적지 노드를 찾지 못했습니다. 맵/상태를 확인하세요.");
                StartCoroutine(RetryAssignLater());
                return;
            }
        }

        // 3) 시작/목표 Walkable 보장 (동적 차단 과보정 대비)
        if (forceStartGoalWalkable)
        {
            _start.Walkable = true;
            _end.Walkable = true;
        }

        // 4) Path 요청
        Managers.Path.RequestPath(_start, _end, route =>
        {
            if (route == null || route.Count == 0)
            {
                Debug.LogWarning("[VirtualNavigator] 경로 생성 실패. 잠시 후 재시도합니다.");
                StartCoroutine(RetryAssignLater());
            }
            else
            {
                // VirtualDeviceController가 내부적으로
                // "정지→차단→재탐색" 및 교착 해소(우선순위/후퇴)를 수행
                _deviceController.MoveDevice(route, AssignTask);
            }
        });
    }

    /// <summary> GridSafeUtil을 감싼 헬퍼(링 파라미터 전달). </summary>
    private bool TryGetNearestWalkable(Vector3 worldPos, out Node node, int maxRing)
    {
        // GridSafeUtil.TryGetNearestWalkableNode는 maxRing 인자를 직접 받지 않으므로
        // 내부 maxRing을 바꿔 쓰고 싶다면, 아래처럼 간단 래핑을 유지
        bool ok = GridSafeUtil.TryGetNearestWalkableNode(worldPos, out node, maxRing);
        return ok && node != null && node.Walkable;
    }

    private IEnumerator RetryAssignLater()
    {
        yield return new WaitForSeconds(retryDelay);
        AssignTask();
    }
}
