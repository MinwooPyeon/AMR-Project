using UnityEngine;

public class StateData : MonoBehaviour
{
<<<<<<< HEAD
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
=======
    float _speed = 0;

    AMR_STATE _state = AMR_STATE.RUNNING;
    ACTION_STATE _actionState = ACTION_STATE.STOP;
    public string SerialNumber
    {
        get { return this.gameObject.GetInstanceID().ToString(); }
>>>>>>> origin/develop
    }
    
    public Vector2 Position
    {
<<<<<<< HEAD
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
=======
        get { return new Vector2(transform.position.x, transform.position.z); }
    }

    public float Angle
    {
        get { return transform.rotation.y; }
    }
    public float Speed
    {
        get { return _speed; }
        set { _speed = value; }
>>>>>>> origin/develop
    }
    public AMR_STATE AmrState
    {
        get { return _state; }
        set { _state = value; }
    }
<<<<<<< HEAD

=======
>>>>>>> origin/develop
    public ACTION_STATE ActionState
    {
        get { return _actionState; }
        set { _actionState = value; }
    }
}
