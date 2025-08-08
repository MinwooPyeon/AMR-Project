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

    private const float AngleThreshold = 1f;      // �� ����
    private const float PositionThreshold = 0.1f; // ���� ����
    #endregion

    #region Public API
    /// <summary>
    /// ��� ����Ʈ�� �Ϸ� �ݹ��� �޾� AMR�� �̵���ŵ�ϴ�.
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
            // 1) ��ǥ �������� ȸ��
            yield return StartCoroutine(RotateTo(targetPos));

            // 2) ��ǥ �������� �̵�
            yield return StartCoroutine(Approach(targetPos));

            // 3) ����
            StopMovement();
            yield return null;
        }
        // ��� ��� ��ȸ �� �Ϸ� �ݹ� ȣ��
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

            // ȸ���� ���� ������ �� ������ ���¸� üũ
            while (_state.ActionState != ACTION_STATE.STOP)
                yield return null;

            angle = GetAngleToTarget(targetPos);
            Debug.Log(angle);
        }

        _moveController.Stop();  // Stop() ���� ActionState�� �ٲ�� �����߱� ������
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
