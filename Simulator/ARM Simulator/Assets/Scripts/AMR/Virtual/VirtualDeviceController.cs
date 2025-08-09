using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ��� ��ȸ �� MoveController ���� ���ڸ� ȸ�� �� ���� �̵������� AMR�� �����մϴ�.
/// </summary>
public class VirtualDeviceController : MonoBehaviour
{
    [Header("���ڸ� ȸ�� �ӵ� (deg/s)")]
    public float rotateSpeed = 180f;
    [Header("���� �̵� �ӵ� (units/s)")]
    public float moveSpeed = 5f;

    private List<Node> _route;
    private Action _onRouteComplete;

    private const float AngleThreshold = 1f;   // �� ����
    private const float PositionThreshold = 0.1f; // ���� ����

    /// <summary>
    /// ��� ����Ʈ�� �Ϸ� �ݹ��� �޾� ���������� ���ڸ� ȸ�� & ���� �̵��� �����մϴ�.
    /// </summary>
    public void MoveDevice(List<Node> route, Action onComplete = null)
    {
        _route = route;
        _onRouteComplete = onComplete;
        StopAllCoroutines();
        StartCoroutine(ProcessRoute());
    }

    private IEnumerator ProcessRoute()
    {
        foreach (var node in _route)
        {
            // ��ǥ ���� ��ǥ (Y ���� ���� ����)
            Vector3 targetPos = CoordinateCalc.GridToWorld(node.Pos, 1, Managers.Map.Resolution);

            // 1) ���ڸ� ȸ��
            yield return StartCoroutine(RotateToFace(targetPos));
            // 2) ���� �̵�
            yield return StartCoroutine(MoveToPoint(targetPos));
        }

        // �Ϸ� �ݹ�
        _onRouteComplete?.Invoke();
    }
    /// <summary>
    /// ���ڸ����� ��ǥ ������ �ٶ󺸵��� ȸ���մϴ�.
    /// </summary>
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

    /// <summary>
    /// ��ǥ �������� MoveSpeed �ӵ��� ���� �̵��մϴ�.
    /// </summary>
    private IEnumerator MoveToPoint(Vector3 target)
    {
        while (Vector3.Distance(transform.position, target) > PositionThreshold)
        {
            transform.position = Vector3.MoveTowards(
                transform.position,
                target,
                moveSpeed * Time.deltaTime);
            yield return null;
        }
    }
}
