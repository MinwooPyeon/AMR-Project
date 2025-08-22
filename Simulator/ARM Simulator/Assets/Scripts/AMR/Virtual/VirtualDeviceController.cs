using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ���ڸ� ȸ�� + ���� ���� ��Ʈ�ѷ�.
/// ���� ��ġ/���忣�� ���� ���� �ؼ�:
/// - SerialNumber(�ϰ� ��Ģ) + 'Ż�� ���(BFS)'���� ����/�纸�� ����
/// - ����: ���� ���� �� ��ǥ�� ��Ž��
/// - �纸��: ��� �� '���� ����'���� ���� �� ��� ��� �� ����ǥ ��Ž��
/// </summary>
public class VirtualDeviceController : MonoBehaviour
{
    // ===== ���� �Ķ���� =====
    [Header("ȸ��/����")]
    public float rotateSpeed = 180f;
    public float moveSpeed = 5f;

    // ===== ����/�Ǵ� =====
    [Header("����/�Ǵ�")]
    public float detectRadius = 1.2f;   // �浹 ���� �ݰ�
    public float forwardBias = 0.5f;   // ���� ������
    public float safeDistance = 0.6f;   // ���� �Ÿ�
    public LayerMask amrLayer;          // AMR ���� ���̾�

    // ===== ����/��Ž�� =====
    [Header("����/��Ž��")]
    public float blockRadius = 0.6f;   // ���� ���� �ݰ�
    public float blockDuration = 1.2f;   // ���� ���� �ð�
    public float replanCooldown = 0.8f;   // ��Ž�� ��ٿ�
    public bool useSegmentBlocking = false; // ������ �� �κ� ���� ���� ���� ���

    // ===== �켱����/���� =====
    [Header("�켱����/����")]
    public bool smallerSerialWins = true; // ���� �������� ����
    public float yieldHoldTime = 2.0f; // �纸 ���� �� ���
    [Tooltip("���� ���� �Ǵܿ� ���� �ݰ�(m): �κ� ������ ���� ũ��")]
    public float openClearance = 0.7f;
    [Tooltip("BFS �ִ� Ž�� �� ��(��� ���̺��� ũ��)")]
    public int maxEscapeSteps = 160;
    [Tooltip("��� �ǵ����� ��� �� �ִ� �齺��(�ɼ�)")]
    public int maxBackSteps = 60;

    // ===== ���� ���� =====
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

    // ===== ���� API =====
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

    // ===== ����������Ŭ =====
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

    // ===== ��� ó�� ���� =====
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

            // 1) ���ڸ� ȸ��
            yield return StartCoroutine(RotateToFace(targetPos));

            // 2) ����(���� �ؼ� ����)
            bool replanTriggered = false;
            yield return StartCoroutine(MoveToPoint(targetPos, () => replanTriggered = true));
            if (replanTriggered) yield break; // �� ��η� ��ü��
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

    /// �̵� �� �浹 ���� ��: Ż�� ���ɼ�(BFS) + SerialNumber�� ����/�纸 ����
    private IEnumerator MoveToPoint(Vector3 target, Action replanFlag)
    {
        while (Vector3.Distance(transform.position, target) > PositionThreshold)
        {
            // �纸 ��� ���̸� ����
            if (_isYielding && Time.time < _yieldUntil)
            {
                yield return null;
                continue;
            }

            // 1) ���� ����(���� ������ ���� ���� ����)
            bool danger = SenseOpponent(out Transform opp, out Vector3 contactCenter);
            if (danger && opp != null)
            {
                // 2) ��/��� ���� ��� Ȯ��
                GridSafeUtil.TryGetNearestWalkableNode(transform.position, out Node myStart);
                GridSafeUtil.TryGetNearestWalkableNode(opp.position, out Node opStart);

                // 3) Ż�� ���ɼ�/���(BFS steps) ���
                bool myCan = GridSafeUtil.TryFindNearestOpenBFS(myStart, openClearance, maxEscapeSteps, out Node myOpen, out int mySteps);
                bool opCan = GridSafeUtil.TryFindNearestOpenBFS(opStart, openClearance, maxEscapeSteps, out Node opOpen, out int opSteps);

                // 4) ����/�纸 ����
                bool iYield; // true=���� �纸
                if (!myCan && opCan) iYield = false;                        // ���� �� ���� �� ���� ����
                else if (myCan && !opCan) iYield = true;                         // ��밡 �� ���� �� ���� �纸
                else if (myCan && opCan) iYield = (mySteps + 2 < opSteps);      // �� �� ���� �纸(�����׸��ý� 2)
                else iYield = !IsLeaderAgainst(opp);        // �� �� �Ұ� �� SerialNumber Ÿ�̺극��Ŀ

                if (iYield)
                {
                    // === �纸��: ���� �������� ���� �� ��� �� ����ǥ ��Ž�� ===
                    _isYielding = true;

                    // (�ɼ�) ���� ��θ� �Ųٷ� Ÿ�� ���� ���� ã�� ���� �� �װ� �켱 ���
                    if (!GridSafeUtil.TryFindOpenAlongRouteBackwards(_route, _routeIndex, openClearance, maxBackSteps, out Node backOpen))
                        backOpen = myOpen; // ������ BFS ��� ���

                    ReplanTo(backOpen, () =>
                    {
                        _yieldUntil = Time.time + yieldHoldTime;
                        ReplanFromHere(); // _goalWorld ����
                        _isYielding = false;
                    });
                    replanFlag?.Invoke();
                    yield break;
                }
                else
                {
                    // === ����: ���� ���� �� ���� ������ ��Ž�� ===
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
                // ����: ����
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

    // ===== ����/�켱���� =====

    /// �� ���� �����ݰ� ���� ���� �������� AMR�� ã��(����/����� ����)
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

    /// SerialNumber�� �ϰ��� �켱����(���� ����) ����
    private bool IsLeaderAgainst(Transform opponent)
    {
        string mySN = _state != null ? _state.SerialNumber : GetInstanceID().ToString();
        string opSN = opponent.GetComponent<StateData>()?.SerialNumber ?? opponent.GetInstanceID().ToString();

        bool myParsed = long.TryParse(mySN, out long myNum);
        bool opParsed = long.TryParse(opSN, out long opNum);

        int cmp = (myParsed && opParsed)
            ? myNum.CompareTo(opNum)
            : string.Compare(mySN, opSN, StringComparison.Ordinal);

        // smallerSerialWins=true �� ����/������ '���� ��'�� ����
        return smallerSerialWins ? (cmp < 0) : (cmp > 0);
    }

    // ===== ��Ž�� ���� =====

    /// ���� ��ġ(start) �� _goalWorld(goal) ��Ž��
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
            Debug.LogWarning($"[VirtualDeviceController] ReplanFromHere ����: {e.Message}");
        }
    }

    /// Ư�� ���(goal)�� �ӽ� ���(���� ��) ��û
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
            Debug.LogWarning($"[VirtualDeviceController] ReplanTo ����: {e.Message}");
        }
    }

    private IEnumerator RetryReplanDelayed(float delay)
    {
        yield return new WaitForSeconds(delay);
        ReplanFromHere();
    }
}
