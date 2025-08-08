using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualDeviceController : MonoBehaviour
{
    #region Attributes
    private MoveController _moveController;
    private StateData _state;
    private List<Node> _route;
    private Action _onRouteComplete;

    private const float AngleThreshold = 1f;      // 도 단위
    private const float PositionThreshold = 0.1f; // 월드 단위
    #endregion

    #region Public API
    /// <summary>
    /// 경로 리스트와 완료 콜백을 받아 AMR을 이동시킵니다.
    /// </summary>
    public void MoveDevice(List<Node> route, Action onComplete = null)
    {
        _route = route;
        _onRouteComplete = onComplete;
        StartCoroutine(ProcessRoute());
    }
    #endregion

    #region Coroutine Steps
    private IEnumerator ProcessRoute()
    {
        int cnt = _route.Count;
        //Debug.Log(_route[cnt - 1].Pos);
        for(int i =0;i<cnt;i++)
        {
            Vector2 routePos = _route[i].Pos;
            Vector3 targetPos = new Vector3(routePos.x, 1, routePos.y);
            //Debug.Log(targetPos);
            // 1) 목표 방향으로 회전
            yield return StartCoroutine(RotateTo(targetPos));

            // 2) 목표 지점까지 이동
            yield return StartCoroutine(Approach(targetPos));

            // 3) 정지
            StopMovement();
            yield return null;
        }
        // 모든 노드 순회 후 완료 콜백 호출
        _onRouteComplete?.Invoke();
    }

    private IEnumerator RotateTo(Vector3 targetPos)
    {
        float angle = GetAngleToTarget(targetPos);
        while (Mathf.Abs(angle) > AngleThreshold)
        {
            if (angle > 0)
                _moveController.RotateRight();
            else
                _moveController.RotateLeft();

            // 회전이 끝날 때까지 매 프레임 상태만 체크
            while (_state.ActionState != ACTION_STATE.STOP)
                yield return null;

            angle = GetAngleToTarget(targetPos);
            Debug.Log(angle);
        }

        _moveController.Stop();  // Stop() 에서 ActionState도 바뀌도록 수정했기 때문에
    }


    private IEnumerator Approach(Vector3 targetPos)
    {
        Debug.Log(Vector3.Distance(_state.WorldPosition, targetPos));
        while (Vector3.Distance(_state.WorldPosition, targetPos) > PositionThreshold)
        {
            _moveController.MoveForward(_moveController.MoveSpeed);
            yield return null;
        }
        StopMovement();
    }
    #endregion

    #region Movement Helpers
    private void StopMovement()
    {
        _moveController.Stop();
    }
    #endregion

    #region Utility
    private float GetAngleToTarget(Vector3 targetWorldPos)
    {
        Vector3 toTarget = (targetWorldPos - transform.position).normalized;
        return Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
    }
    #endregion

    #region Unity Methods
    private void Start()
    {
        _moveController = GetComponent<MoveController>();
        _state = GetComponent<StateData>();

        Managers.Device.RegistVirtualDevice(gameObject.GetInstanceID().ToString(), this);
    }

    private void OnDestroy()
    {
        Managers.Device.DeviceUnregister(gameObject.GetInstanceID().ToString());
    }
    #endregion
}
