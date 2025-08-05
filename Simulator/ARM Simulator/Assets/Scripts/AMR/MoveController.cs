using System.Collections;
using UnityEngine;

public class MoveController : MonoBehaviour
{
    #region Attribute
    private StateData _stateData;
    [SerializeField] private float _rotateSpeed = 180f;
    private Coroutine activeCoroutine = null;
    #endregion
    #region Public API

    public void MoveForward(float acceleration)
    {
        // 다른 행동 중이면 이동 명령 무시
        if (_stateData.ActionState == ACTION_STATE.ROTATE_LEFT && _stateData.ActionState == ACTION_STATE.ROTATE_RIGHT)
        {
            Debug.LogWarning($"MoveForward ignored: current state is {_stateData.AmrState}");
            return;
        }

        StopActiveCoroutine();

        _stateData.Acceleration = acceleration;
        _stateData.ActionState = ACTION_STATE.MOVE_FORWARD;
        activeCoroutine = StartCoroutine(MoveForwardCoroutine());
    }

    public void MoveBackward(float acceleration)
    {
        // 다른 행동 중이면 이동 명령 무시
        if (_stateData.ActionState == ACTION_STATE.ROTATE_LEFT && _stateData.ActionState == ACTION_STATE.ROTATE_RIGHT)
        {
            Debug.LogWarning($"MoveForward ignored: current state is {_stateData.AmrState}");
            return;
        }

        StopActiveCoroutine();

        _stateData.Acceleration = acceleration;
        _stateData.ActionState = ACTION_STATE.MOVE_BACKWARD;
        activeCoroutine = StartCoroutine(MoveBackwardCoroutine());
    }

    public void Stop(float value = 0)
    {
        StopActiveCoroutine();
        _stateData.AmrState = AMR_STATE.RUNNING;
    }

    public void RotateLeft()
    {
        _stateData.ActionState = ACTION_STATE.ROTATE_LEFT;
        Rotate(-90f);
    }
    public void RotateRight()
    {
        _stateData.ActionState =(ACTION_STATE.ROTATE_RIGHT);
        Rotate(90f);
    }

    private void Rotate(float deltaAngle)
    {
        // 회전 동작 중에는 MoveForward가 실행되지 않음
        StopActiveCoroutine();

        activeCoroutine = StartCoroutine(RotateCoroutine(deltaAngle));
    }

    #endregion
    #region Coroutine Implementations

    private IEnumerator MoveForwardCoroutine()
    {
        while (_stateData.ActionState == ACTION_STATE.MOVE_FORWARD)
        {
            transform.Translate(Vector3.forward * _stateData.Acceleration * Time.deltaTime);
            yield return null;
        }

        activeCoroutine = null;
    }
    private IEnumerator MoveBackwardCoroutine()
    {
        while (_stateData.ActionState == ACTION_STATE.MOVE_BACKWARD)
        {
            transform.Translate(Vector3.back * _stateData.Acceleration * Time.deltaTime);
            yield return null;
        }

        activeCoroutine = null;
    }
    private IEnumerator RotateCoroutine(float deltaAngle)
    {
        Quaternion startRot = transform.rotation;
        Quaternion targetRot = Quaternion.Euler(0f, startRot.eulerAngles.y + deltaAngle, 0f);

        while (Quaternion.Angle(transform.rotation, targetRot) > 0.01f)
        {
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, _rotateSpeed * Time.deltaTime);
            yield return null;
        }

        _stateData.ActionState = ACTION_STATE.STOP;
        activeCoroutine = null;
    }

    #endregion
    #region Helpers

    private void StopActiveCoroutine()
    {
        if (activeCoroutine != null)
        {
            StopCoroutine(activeCoroutine);
            activeCoroutine = null;
        }
    }

    #endregion
    #region Unity Methods
    private void Awake()
    {
        _stateData = GetComponent<StateData>();
    }
    #endregion
}
