using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 경로 순회 시 MoveController 없이 제자리 회전 및 직진 이동만으로 AMR을 제어합니다.
/// </summary>
public class VirtualDeviceController : MonoBehaviour
{
    [Header("제자리 회전 속도 (deg/s)")]
    public float rotateSpeed = 180f;
    [Header("직진 이동 속도 (units/s)")]
    public float moveSpeed = 5f;

    private List<Node> _route;
    private Action _onRouteComplete;

    private const float AngleThreshold = 1f;   // 도 단위
    private const float PositionThreshold = 0.1f; // 유닛 단위

    /// <summary>
    /// 경로 리스트와 완료 콜백을 받아 순차적으로 제자리 회전 & 직진 이동을 수행합니다.
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
            // 목표 월드 좌표 (Y 축은 현재 유지)
            Vector3 targetPos = CoordinateCalc.GridToWorld(node.Pos, 1, Managers.Map.Resolution);

            // 1) 제자리 회전
            yield return StartCoroutine(RotateToFace(targetPos));
            // 2) 직진 이동
            yield return StartCoroutine(MoveToPoint(targetPos));
        }

        // 완료 콜백
        _onRouteComplete?.Invoke();
    }
    /// <summary>
    /// 제자리에서 목표 방향을 바라보도록 회전합니다.
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
    /// 목표 지점까지 MoveSpeed 속도로 직진 이동합니다.
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
