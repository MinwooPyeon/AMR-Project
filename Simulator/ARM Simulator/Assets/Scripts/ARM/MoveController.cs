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
        if (_stateData.ArmState != ARM_STATE.IDLE)
        {
            Debug.LogWarning($"MoveForward ignored: current state is {_stateData.ArmState}");
            return;
        }

        StopActiveCoroutine();

        _stateData.Acceleration = acceleration;
        _stateData.ArmState = ARM_STATE.RUNNING;
        activeCoroutine = StartCoroutine(MoveForwardCoroutine());
    }

    public void Stop(float value = 0)
    {
        StopActiveCoroutine();
        _stateData.ArmState = ARM_STATE.IDLE;
    }

    public void RotateLeft() => Rotate(-90f);
    public void RotateRight() => Rotate(90f);

    private void Rotate(float deltaAngle)
    {
        // 회전 동작 중에는 MoveForward가 실행되지 않음
        StopActiveCoroutine();

        _stateData.ArmState = ARM_STATE.ROTATING;
        activeCoroutine = StartCoroutine(RotateCoroutine(deltaAngle));
    }

    #endregion
    #region Coroutine Implementations

    private IEnumerator MoveForwardCoroutine()
    {
        while (_stateData.ArmState == ARM_STATE.RUNNING)
        {
            transform.Translate(Vector3.forward * _stateData.Acceleration * Time.deltaTime);
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

        _stateData.ArmState = ARM_STATE.IDLE;
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
