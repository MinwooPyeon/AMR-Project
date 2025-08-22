using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeviceController : MonoBehaviour
{
    #region Attribute
    [Header("제자리 회전 속도 (deg/s)")]
    public float rotateSpeed = 180f;

    [Header("직진 이동 속도 (units/s)")]
    public float moveSpeed = 5f;

    private const float AngleThreshold = 1f;       // 도 단위
    private const float PositionThreshold = 0.1f;  // 유닛 단위

    // === New: 명령 큐 & 실행 상태 ===
    private readonly Queue<StatusMsg> _orderQueue = new();
    private bool _isProcessing = false;
    private Coroutine _processOrdersCo = null;

    // 현재 진행 중인 경로(옵션: 모니터링용)
    private List<Node> _currentRoute = null;
    #endregion

    #region Public API (외부에서 호출)
    /// <summary>
    /// 외부에서 들어온 명령(msg)을 큐에 쌓는다.
    /// </summary>
    public void EnqueueOrder(StatusMsg msg)
    {
        _orderQueue.Enqueue(msg);
        TryStartProcessing();
    }

    /// <summary>
    /// (기존) 단일 명령 즉시 수행 트리거가 필요하면 이것도 유지 가능.
    /// 내부적으로는 큐에 적재.
    /// </summary>
    public void ExcuteOrder(StatusMsg msg)
    {
        EnqueueOrder(msg);
    }

    /// <summary>
    /// 큐 비우기 (현재 이동은 유지)
    /// </summary>
    public void ClearQueue()
    {
        _orderQueue.Clear();
    }

    /// <summary>
    /// 즉시 정지 (현재 이동/코루틴 중단 + 큐 비움)
    /// </summary>
    public void StopImmediately()
    {
        _orderQueue.Clear();
        if (_processOrdersCo != null)
        {
            StopCoroutine(_processOrdersCo);
            _processOrdersCo = null;
        }
        StopAllCoroutines(); // 이동/회전 코루틴 중단
        _isProcessing = false;
        _currentRoute = null;
    }

    /// <summary>
    /// 현재 작업만 스킵하고 다음 큐로 진행하고 싶을 때
    /// </summary>
    public void SkipCurrent()
    {
        // 현재 이동을 멈춘 뒤, 주문 루프만 재개
        StopAllCoroutines();
        // 루프는 while에서 다음으로 넘어감
    }
    #endregion

    #region Internal Orchestration
    private void TryStartProcessing()
    {
        if (!_isProcessing)
        {
            _isProcessing = true;
            _processOrdersCo = StartCoroutine(ProcessOrders());
        }
    }

    /// <summary>
    /// 큐에 쌓인 StatusMsg를 한 개씩 꺼내어
    /// 경로 요청 → 이동 수행을 순차 처리
    /// </summary>
    private IEnumerator ProcessOrders()
    {
        while (_orderQueue.Count > 0)
        {
            StatusMsg msg = _orderQueue.Dequeue();

            // 1) 시작/도착 노드 계산
            Node start = Managers.Map.Grid.GetLocateNode(this.transform.position);
            // msg.position은 그리드 좌표라고 가정 (x,y)
            Vector3 targetWorld = CoordinateCalc.GridToWorld(
                new Vector2Int((int)msg.position.x, (int)msg.position.y),
                Managers.Map.Height, Managers.Map.Resolution
            );
            Node end = Managers.Map.Grid.GetLocateNode(targetWorld);

            // 2) 경로 요청 (콜백 기반)
            List<Node> route = null;
            bool pathDone = false;

            // NOTE: RequestPath가 콜백을 받도록 가정 (onComplete: List<Node>)
            Managers.Path.RequestPath(start, end, (r) =>
            {
                route = r;
                pathDone = true;
            });

            // 3) 경로 탐색 완료 대기 (타임아웃 옵션 부여)
            float timeout = 5f; // 필요시 조절
            float t = 0f;
            yield return new WaitUntil(() =>
            {
                t += Time.deltaTime;
                return pathDone || t >= timeout;
            });

            if (!pathDone || route == null || route.Count == 0)
            {
                Debug.LogWarning("[DeviceController] 경로 탐색 실패 또는 타임아웃. 다음 명령으로 넘어갑니다.");
                continue; // 다음 큐 항목 진행
            }

            _currentRoute = route;

            // 4) 경로에 따라 이동 수행 (회전 → 직진)
            yield return StartCoroutine(WalkRoute(route));

            _currentRoute = null;

            // (선택) 한 건 완료 후 작은 딜레이
            yield return null;
        }

        _isProcessing = false;
        _processOrdersCo = null;
    }
    #endregion

    #region Movement (기존 로직 재사용)
    /// <summary>
    /// 기존 MoveDevice를 내부용으로 바꿔, 외부에서 코루틴을 중첩 시작하지 않게 함.
    /// </summary>
    private IEnumerator WalkRoute(List<Node> route)
    {
        foreach (var node in route)
        {
            Vector3 targetPos = CoordinateCalc.GridToWorld(node.Pos, 1, Managers.Map.Resolution);
            // 1) 회전
            yield return StartCoroutine(RotateToFace(targetPos));
            // 2) 이동
            yield return StartCoroutine(MoveToPoint(targetPos));
        }
    }

    private IEnumerator RotateToFace(Vector3 target)
    {
        while (true)
        {
            Vector3 direction = target - transform.position;
            direction.y = 0;
            if (direction.sqrMagnitude < 0.0001f)
                yield break;

            float angle = Vector3.SignedAngle(transform.forward, direction, Vector3.up);
            if (Mathf.Abs(angle) < AngleThreshold)
                yield break;

            float step = rotateSpeed * Time.deltaTime;
            float turn = Mathf.Clamp(angle, -step, step);
            transform.Rotate(0f, turn, 0f);
            yield return null;
        }
    }

    private IEnumerator MoveToPoint(Vector3 target)
    {
        while (Vector3.Distance(transform.position, target) > PositionThreshold)
        {
            transform.position = Vector3.MoveTowards(
                transform.position,
                target,
                moveSpeed * Time.deltaTime
            );
            yield return null;
        }
    }
    #endregion

    #region Unity Methods
    private void OnDestroy()
    {
        Managers.Device.DeviceUnregister(this.gameObject.GetInstanceID().ToString());
    }
    #endregion
}
