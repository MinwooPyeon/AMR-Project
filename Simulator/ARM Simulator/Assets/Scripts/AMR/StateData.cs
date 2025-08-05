using UnityEngine;

public class StateData : MonoBehaviour
{
    float _acceleration = 0;
    float _chargeAmount = 0;
    Vector2 _position = Vector2.zero;
    string _serialNumber;

    AMR_STATE _state = AMR_STATE.IDLE;
    ACTION_STATE _actionState = ACTION_STATE.STOP;

    public string SerialNumber
    {
        get { return _serialNumber; }
        set { _serialNumber = value; }
    }
    
    public Vector2 Position
    {
        get { return _position; }
        set { _position = value; }
    }
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
    public AMR_STATE AmrState
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
