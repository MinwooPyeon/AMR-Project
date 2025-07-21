using UnityEngine;

public class StateData : MonoBehaviour
{
    float _acceleration = 0;
    float _chargeAmount = 0;
    ARM_STATE _state = ARM_STATE.IDLE;

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
    public ARM_STATE State
    {
        get { return _state; }
        set { _state = value; }
    }
}
