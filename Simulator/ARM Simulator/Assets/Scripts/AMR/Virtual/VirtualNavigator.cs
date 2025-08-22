using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualNavigator : MonoBehaviour
{
    [Header("Refs")]
    public StateData _state;
    public VirtualDeviceController _deviceController;

    [Header("�ɼ�")]
    public bool forceStartGoalWalkable = true; // ����/��ǥ�� �׻� Walkable�� ����
    public bool useTransformFallback = true; // _state.WorldPosition�� ��ȿ���� ������ transform.position ���
    public float retryDelay = 0.5f; // ���� �� ��õ� ����(��)
    public int maxStartSearchRing = 25;   // ������ ���� �� �ִ� ��(��)
    public int maxGoalSearchRing = 25;   // ��ǥ�� ���� �� �ִ� ��(��)

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

    /// <summary> ���¿� ���� �������� �����ϰ� ��� ��û�� ����. </summary>
    public void AssignTask()
    {
        _end = GetEndPosition();
        if (_end == null)
        {
            Debug.LogWarning("[VirtualNavigator] ������ ��尡 null�Դϴ�. ����/�� ���� Ȯ�� �� ��õ��մϴ�.");
            StartCoroutine(RetryAssignLater());
            return;
        }

        RequestAndMove();
    }

    /// <summary> AMR ���¿� ���� ������ ��� ����. </summary>
    private Node GetEndPosition()
    {
        // Managers.Map.GetRandomLoaderPos() / GetRandomDroperPos() �� Node�� ��ȯ�Ѵٰ� ����.
        if (_state == null)
        {
            // ���������� ������ �⺻ ������(Loader)�� ����. ������Ʈ ��å�� �°� ���� ����.
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
                Debug.LogWarning($"[VirtualNavigator] ���� ����: {_state.AmrState} �� �⺻ ������(Loader) ���");
                return Managers.Map.GetRandomLoaderPos();
        }
    }

    /// <summary> ����/��ǥ ��带 �����ϰ� �����ϰ� Path�� ��û�� �� ���� ����. </summary>
    private void RequestAndMove()
    {
        // 1) ���� ��� ���� ȹ��
        Vector3 worldStart = Vector3.zero;

        if (_state != null && _state.WorldPosition != Vector3.zero)
            worldStart = _state.WorldPosition;
        else if (useTransformFallback)
            worldStart = transform.position;
        else
            worldStart = transform.position;

        if (!TryGetNearestWalkable(worldStart, out _start, maxStartSearchRing))
        {
            Debug.LogError($"[VirtualNavigator] ���� ��带 ã�� ���߽��ϴ�. worldStart={worldStart}");
            StartCoroutine(RetryAssignLater());
            return;
        }

        // 2) ������ ��� ���� (null �Ǵ� ���Ŀ���̸� ��ó Walkable�� ����)
        if (_end == null || !_end.Walkable)
        {
            // _end�� null�� ���� ������, ������ �� ������ǥ�� ��ȯ �� ��ó Walkable Ž��
            Vector3 goalWorld = (_end != null)
                ? CoordinateCalc.GridToWorld(_end.Pos, Managers.Map.Height, Managers.Map.Resolution)
                : transform.position; // fallback (�����δ� �� ���̽� ���� ����)

            if (!TryGetNearestWalkable(goalWorld, out _end, maxGoalSearchRing))
            {
                Debug.LogError("[VirtualNavigator] ������ ��带 ã�� ���߽��ϴ�. ��/���¸� Ȯ���ϼ���.");
                StartCoroutine(RetryAssignLater());
                return;
            }
        }

        // 3) ����/��ǥ Walkable ���� (���� ���� ������ ���)
        if (forceStartGoalWalkable)
        {
            _start.Walkable = true;
            _end.Walkable = true;
        }

        // 4) Path ��û
        Managers.Path.RequestPath(_start, _end, route =>
        {
            if (route == null || route.Count == 0)
            {
                Debug.LogWarning("[VirtualNavigator] ��� ���� ����. ��� �� ��õ��մϴ�.");
                StartCoroutine(RetryAssignLater());
            }
            else
            {
                // VirtualDeviceController�� ����������
                // "���������ܡ���Ž��" �� ���� �ؼ�(�켱����/����)�� ����
                _deviceController.MoveDevice(route, AssignTask);
            }
        });
    }

    /// <summary> GridSafeUtil�� ���� ����(�� �Ķ���� ����). </summary>
    private bool TryGetNearestWalkable(Vector3 worldPos, out Node node, int maxRing)
    {
        // GridSafeUtil.TryGetNearestWalkableNode�� maxRing ���ڸ� ���� ���� �����Ƿ�
        // ���� maxRing�� �ٲ� ���� �ʹٸ�, �Ʒ�ó�� ���� ������ ����
        bool ok = GridSafeUtil.TryGetNearestWalkableNode(worldPos, out node, maxRing);
        return ok && node != null && node.Walkable;
    }

    private IEnumerator RetryAssignLater()
    {
        yield return new WaitForSeconds(retryDelay);
        AssignTask();
    }
}
