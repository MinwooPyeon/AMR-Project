using UnityEngine;

public class StateData : MonoBehaviour
{
    float _acceleration = 0;
    float _chargeAmount = 0;
    ARM_STATE _state = ARM_STATE.IDLE;
    ACTION_STATE _actionState = ACTION_STATE.STOP;

    public float Acceleration
    {
        get { return _acceleration; }
        set { _acceleration = value; }
    }
    public float ChargeAmount
    {
        get { return _chargeAmount; }
        set { _chargeAmount = value; }
    }
    public ARM_STATE ArmState
    {
        get { return _state; }
        set { _state = value; }
    }

    public ACTION_STATE ActionState
    {
        get { return _actionState; }
        set { _actionState = value; }
    }
}
