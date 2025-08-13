using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 제자리 회전 + 직진 주행 컨트롤러.
/// 정면 대치/데드엔드 포함 교착 해소:
/// - SerialNumber(일관 규칙) + '탈출 비용(BFS)'으로 리더/양보자 결정
/// - 리더: 현장 차단 → 목표로 재탐색
/// - 양보자: 통로 밖 '오픈 지점'까지 후퇴 → 잠시 대기 → 원목표 재탐색
/// </summary>
public class VirtualDeviceController : MonoBehaviour
{
    // ===== 주행 파라미터 =====
    [Header("회전/직진")]
    public float rotateSpeed = 180f;
    public float moveSpeed = 5f;

    // ===== 감지/판단 =====
    [Header("감지/판단")]
    public float detectRadius = 1.2f;   // 충돌 감지 반경
    public float forwardBias = 0.5f;   // 전방 오프셋
    public float safeDistance = 0.6f;   // 위험 거리
    public LayerMask amrLayer;          // AMR 전용 레이어

    // ===== 차단/재탐색 =====
    [Header("차단/재탐색")]
    public float blockRadius = 0.6f;   // 현장 차단 반경
    public float blockDuration = 1.2f;   // 차단 유지 시간
    public float replanCooldown = 0.8f;   // 재탐색 쿨다운
    public bool useSegmentBlocking = false; // 리더가 두 로봇 사이 선분 차단 사용

    // ===== 우선순위/후퇴 =====
    [Header("우선순위/후퇴")]
    public bool smallerSerialWins = true; // 숫자 작을수록 리더
    public float yieldHoldTime = 2.0f; // 양보 도착 후 대기
    [Tooltip("오픈 지점 판단용 여유 반경(m): 로봇 폭보다 조금 크게")]
    public float openClearance = 0.7f;
    [Tooltip("BFS 최대 탐색 셀 수(통로 길이보다 크게)")]
    public int maxEscapeSteps = 160;
    [Tooltip("경로 되돌리기 사용 시 최대 백스텝(옵션)")]
    public int maxBackSteps = 60;

    // ===== 내부 상태 =====
    private List<Node> _route;
    private Action _onRouteComplete;
    private Vector3 _goalWorld;

    private float _lastReplanTime = -999f;
    private bool _isYielding = false;
    private float _yieldUntil = -1f;

    private int _routeIndex = 0;

    private const float AngleThreshold = 1f;
    private const float PositionThreshold = 0.1f;

    private static readonly Collider[] _neighborsBuf = new Collider[16];
    private StateData _state; // SerialNumber

    // ===== 공개 API =====
    public void MoveDevice(List<Node> route, Action onComplete = null)
    {
        _route = route;
        _onRouteComplete = onComplete;
        _goalWorld = (route != null && route.Count > 0)
            ? CoordinateCalc.GridToWorld(route[route.Count - 1].Pos, 1, Managers.Map.Resolution)
            : transform.position;

        _isYielding = false;
        _yieldUntil = -1f;

        StopAllCoroutines();
        StartCoroutine(ProcessRoute());
    }

    // ===== 라이프사이클 =====
    private void Start()
    {
        Managers.Device.RegistVirtualDevice(this);
        _state = GetComponent<StateData>();

        if (DynamicObstacle.CoroutineHost == null)
            DynamicObstacle.CoroutineHost = this;
    }

    private void OnDestroy()
    {
        Managers.Device.VirtualDeviceUnregister(GetComponent<StateData>().SerialNumber);
    }

    // ===== 경로 처리 루프 =====
    private IEnumerator ProcessRoute()
    {
        if (_route == null || _route.Count == 0)
        {
            _onRouteComplete?.Invoke();
            yield break;
        }

        for (_routeIndex = 0; _routeIndex < _route.Count; _routeIndex++)
        {
            var node = _route[_routeIndex];
            Vector3 targetPos = CoordinateCalc.GridToWorld(node.Pos, 1, Managers.Map.Resolution);

            // 1) 제자리 회전
            yield return StartCoroutine(RotateToFace(targetPos));

            // 2) 직진(교착 해소 내장)
            bool replanTriggered = false;
            yield return StartCoroutine(MoveToPoint(targetPos, () => replanTriggered = true));
            if (replanTriggered) yield break; // 새 경로로 교체됨
        }

        _onRouteComplete?.Invoke();
    }

    private IEnumerator RotateToFace(Vector3 target)
    {
        while (true)
        {
            Vector3 dir = target - transform.position; dir.y = 0f;
            if (dir.sqrMagnitude < 0.0001f) yield break;

            float angle = Vector3.SignedAngle(transform.forward, dir, Vector3.up);
            if (Mathf.Abs(angle) < AngleThreshold) yield break;

            float step = rotateSpeed * Time.deltaTime;
            float turn = Mathf.Clamp(angle, -step, step);
            transform.Rotate(0f, turn, 0f);
            yield return null;
        }
    }

    /// 이동 중 충돌 감지 시: 탈출 가능성(BFS) + SerialNumber로 리더/양보 결정
    private IEnumerator MoveToPoint(Vector3 target, Action replanFlag)
    {
        while (Vector3.Distance(transform.position, target) > PositionThreshold)
        {
            // 양보 대기 중이면 정지
            if (_isYielding && Time.time < _yieldUntil)
            {
                yield return null;
                continue;
            }

            // 1) 위험 감지(가장 위험한 상대와 접점 추정)
            bool danger = SenseOpponent(out Transform opp, out Vector3 contactCenter);
            if (danger && opp != null)
            {
                // 2) 나/상대 시작 노드 확보
                GridSafeUtil.TryGetNearestWalkableNode(transform.position, out Node myStart);
                GridSafeUtil.TryGetNearestWalkableNode(opp.position, out Node opStart);

                // 3) 탈출 가능성/비용(BFS steps) 계산
                bool myCan = GridSafeUtil.TryFindNearestOpenBFS(myStart, openClearance, maxEscapeSteps, out Node myOpen, out int mySteps);
                bool opCan = GridSafeUtil.TryFindNearestOpenBFS(opStart, openClearance, maxEscapeSteps, out Node opOpen, out int opSteps);

                // 4) 리더/양보 결정
                bool iYield; // true=내가 양보
                if (!myCan && opCan) iYield = false;                        // 나는 못 빠짐 → 내가 리더
                else if (myCan && !opCan) iYield = true;                         // 상대가 못 빠짐 → 내가 양보
                else if (myCan && opCan) iYield = (mySteps + 2 < opSteps);      // 더 싼 쪽이 양보(히스테리시스 2)
                else iYield = !IsLeaderAgainst(opp);        // 둘 다 불가 → SerialNumber 타이브레이커

                if (iYield)
                {
                    // === 양보자: 오픈 지점으로 후퇴 → 대기 → 원목표 재탐색 ===
                    _isYielding = true;

                    // (옵션) 현재 경로를 거꾸로 타서 오픈 지점 찾기 성공 시 그걸 우선 사용
                    if (!GridSafeUtil.TryFindOpenAlongRouteBackwards(_route, _routeIndex, openClearance, maxBackSteps, out Node backOpen))
                        backOpen = myOpen; // 없으면 BFS 결과 사용

                    ReplanTo(backOpen, () =>
                    {
                        _yieldUntil = Time.time + yieldHoldTime;
                        ReplanFromHere(); // _goalWorld 복귀
                        _isYielding = false;
                    });
                    replanFlag?.Invoke();
                    yield break;
                }
                else
                {
                    // === 리더: 현장 차단 → 같은 프레임 재탐색 ===
                    if (Time.time - _lastReplanTime >= replanCooldown)
                    {
                        _lastReplanTime = Time.time;

                        if (useSegmentBlocking)
                            DynamicObstacleEx.MarkSegmentTemporary(transform.position, opp.position, blockRadius, blockDuration);
                        else
                            DynamicObstacle.MarkCircleTemporary(contactCenter, blockRadius, blockDuration);

                        ReplanFromHere();
                        replanFlag?.Invoke();
                        yield break;
                    }
                }
            }
            else
            {
                // 안전: 전진
                Vector3 toTarget = target - transform.position; toTarget.y = 0f;
                Vector3 moveDir = toTarget.normalized;
                transform.position = Vector3.MoveTowards(
                    transform.position,
                    transform.position + moveDir,
                    moveSpeed * Time.deltaTime
                );
            }

            yield return null;
        }
    }

    // ===== 감지/우선순위 =====

    /// 내 앞쪽 안전반경 내의 가장 위협적인 AMR을 찾음(정면/가까움 가중)
    private bool SenseOpponent(out Transform opponent, out Vector3 contactCenterWorld)
    {
        opponent = null;
        contactCenterWorld = transform.position + transform.forward * Mathf.Max(0.2f, forwardBias);

        int count = Physics.OverlapSphereNonAlloc(
            transform.position + transform.forward * forwardBias,
            detectRadius,
            _neighborsBuf,
            amrLayer,
            QueryTriggerInteraction.Ignore
        );
        if (count <= 0) return false;

        Vector3 fwd = transform.forward;
        float best = float.NegativeInfinity;
        bool danger = false;

        for (int i = 0; i < count; i++)
        {
            var c = _neighborsBuf[i];
            if (c == null || c.transform == transform) continue;

            Vector3 r = c.transform.position - transform.position; r.y = 0f;
            float dist = r.magnitude; if (dist < 1e-3f) continue;

            float frontness = Mathf.Clamp01(Vector3.Dot(fwd, r.normalized));
            if (frontness < 0.2f) continue;

            if (dist < safeDistance) danger = true;

            float score = frontness + 1f / Mathf.Max(dist, 0.001f);
            if (score > best)
            {
                best = score;
                opponent = c.transform;
                contactCenterWorld = c.transform.position;
            }
        }
        return opponent != null && danger;
    }

    /// SerialNumber로 일관된 우선순위(리더 여부) 결정
    private bool IsLeaderAgainst(Transform opponent)
    {
        string mySN = _state != null ? _state.SerialNumber : GetInstanceID().ToString();
        string opSN = opponent.GetComponent<StateData>()?.SerialNumber ?? opponent.GetInstanceID().ToString();

        bool myParsed = long.TryParse(mySN, out long myNum);
        bool opParsed = long.TryParse(opSN, out long opNum);

        int cmp = (myParsed && opParsed)
            ? myNum.CompareTo(opNum)
            : string.Compare(mySN, opSN, StringComparison.Ordinal);

        // smallerSerialWins=true → 숫자/사전순 '작은 쪽'이 리더
        return smallerSerialWins ? (cmp < 0) : (cmp > 0);
    }

    // ===== 재탐색 헬퍼 =====

    /// 현재 위치(start) → _goalWorld(goal) 재탐색
    private void ReplanFromHere()
    {
        try
        {
            Node s = Managers.Map.Grid.GetLocateNode(transform.position);
            Node g = Managers.Map.Grid.GetLocateNode(_goalWorld);
            s.Walkable = true; g.Walkable = true;

            Managers.Path.RequestPath(s, g, route =>
            {
                if (route != null && route.Count > 0)
                    MoveDevice(route, _onRouteComplete);
                else
                    StartCoroutine(RetryReplanDelayed(0.4f));
            });
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[VirtualDeviceController] ReplanFromHere 실패: {e.Message}");
        }
    }

    /// 특정 노드(goal)로 임시 경로(후퇴 등) 요청
    private void ReplanTo(Node goal, Action onArrived)
    {
        try
        {
            Node s = Managers.Map.Grid.GetLocateNode(transform.position);
            s.Walkable = true; goal.Walkable = true;

            Managers.Path.RequestPath(s, goal, route =>
            {
                if (route != null && route.Count > 0)
                    MoveDevice(route, onArrived);
                else
                    StartCoroutine(RetryReplanDelayed(0.4f));
            });
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[VirtualDeviceController] ReplanTo 실패: {e.Message}");
        }
    }

    private IEnumerator RetryReplanDelayed(float delay)
    {
        yield return new WaitForSeconds(delay);
        ReplanFromHere();
    }
}
