using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeviceController : MonoBehaviour
{
    #region Attribute
    [Header("���ڸ� ȸ�� �ӵ� (deg/s)")]
    public float rotateSpeed = 180f;

    [Header("���� �̵� �ӵ� (units/s)")]
    public float moveSpeed = 5f;

    private const float AngleThreshold = 1f;       // �� ����
    private const float PositionThreshold = 0.1f;  // ���� ����

    // === New: ��� ť & ���� ���� ===
    private readonly Queue<StatusMsg> _orderQueue = new();
    private bool _isProcessing = false;
    private Coroutine _processOrdersCo = null;

    // ���� ���� ���� ���(�ɼ�: ����͸���)
    private List<Node> _currentRoute = null;
    #endregion

    #region Public API (�ܺο��� ȣ��)
    /// <summary>
    /// �ܺο��� ���� ���(msg)�� ť�� �״´�.
    /// </summary>
    public void EnqueueOrder(StatusMsg msg)
    {
        _orderQueue.Enqueue(msg);
        TryStartProcessing();
    }

    /// <summary>
    /// (����) ���� ��� ��� ���� Ʈ���Ű� �ʿ��ϸ� �̰͵� ���� ����.
    /// ���������δ� ť�� ����.
    /// </summary>
    public void ExcuteOrder(StatusMsg msg)
    {
        EnqueueOrder(msg);
    }

    /// <summary>
    /// ť ���� (���� �̵��� ����)
    /// </summary>
    public void ClearQueue()
    {
        _orderQueue.Clear();
    }

    /// <summary>
    /// ��� ���� (���� �̵�/�ڷ�ƾ �ߴ� + ť ���)
    /// </summary>
    public void StopImmediately()
    {
        _orderQueue.Clear();
        if (_processOrdersCo != null)
        {
            StopCoroutine(_processOrdersCo);
            _processOrdersCo = null;
        }
        StopAllCoroutines(); // �̵�/ȸ�� �ڷ�ƾ �ߴ�
        _isProcessing = false;
        _currentRoute = null;
    }

    /// <summary>
    /// ���� �۾��� ��ŵ�ϰ� ���� ť�� �����ϰ� ���� ��
    /// </summary>
    public void SkipCurrent()
    {
        // ���� �̵��� ���� ��, �ֹ� ������ �簳
        StopAllCoroutines();
        // ������ while���� �������� �Ѿ
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
    /// ť�� ���� StatusMsg�� �� ���� ������
    /// ��� ��û �� �̵� ������ ���� ó��
    /// </summary>
    private IEnumerator ProcessOrders()
    {
        while (_orderQueue.Count > 0)
        {
            StatusMsg msg = _orderQueue.Dequeue();

            // 1) ����/���� ��� ���
            Node start = Managers.Map.Grid.GetLocateNode(this.transform.position);
            // msg.position�� �׸��� ��ǥ��� ���� (x,y)
            Vector3 targetWorld = CoordinateCalc.GridToWorld(
                new Vector2Int((int)msg.position.x, (int)msg.position.y),
                Managers.Map.Height, Managers.Map.Resolution
            );
            Node end = Managers.Map.Grid.GetLocateNode(targetWorld);

            // 2) ��� ��û (�ݹ� ���)
            List<Node> route = null;
            bool pathDone = false;

            // NOTE: RequestPath�� �ݹ��� �޵��� ���� (onComplete: List<Node>)
            Managers.Path.RequestPath(start, end, (r) =>
            {
                route = r;
                pathDone = true;
            });

            // 3) ��� Ž�� �Ϸ� ��� (Ÿ�Ӿƿ� �ɼ� �ο�)
            float timeout = 5f; // �ʿ�� ����
            float t = 0f;
            yield return new WaitUntil(() =>
            {
                t += Time.deltaTime;
                return pathDone || t >= timeout;
            });

            if (!pathDone || route == null || route.Count == 0)
            {
                Debug.LogWarning("[DeviceController] ��� Ž�� ���� �Ǵ� Ÿ�Ӿƿ�. ���� ������� �Ѿ�ϴ�.");
                continue; // ���� ť �׸� ����
            }

            _currentRoute = route;

            // 4) ��ο� ���� �̵� ���� (ȸ�� �� ����)
            yield return StartCoroutine(WalkRoute(route));

            _currentRoute = null;

            // (����) �� �� �Ϸ� �� ���� ������
            yield return null;
        }

        _isProcessing = false;
        _processOrdersCo = null;
    }
    #endregion

    #region Movement (���� ���� ����)
    /// <summary>
    /// ���� MoveDevice�� ���ο����� �ٲ�, �ܺο��� �ڷ�ƾ�� ��ø �������� �ʰ� ��.
    /// </summary>
    private IEnumerator WalkRoute(List<Node> route)
    {
        foreach (var node in route)
        {
            Vector3 targetPos = CoordinateCalc.GridToWorld(node.Pos, 1, Managers.Map.Resolution);
            // 1) ȸ��
            yield return StartCoroutine(RotateToFace(targetPos));
            // 2) �̵�
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
