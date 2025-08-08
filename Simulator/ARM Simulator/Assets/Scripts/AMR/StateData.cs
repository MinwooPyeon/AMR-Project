using UnityEngine;

public class StateData : MonoBehaviour
{
    float _speed = 0;
    string _serial;
    AMR_STATE _state = AMR_STATE.RUNNING;
    ACTION_STATE _actionState = ACTION_STATE.STOP;

    public string SerialNumber
    {
        get { return _serial; }
        set { _serial = value; }
    }
    
    public Vector2 GridPosition
    {
        get { return new Vector2(transform.position.x, transform.position.z); }
    }

    public Vector3 WorldPosition
    {
        get { return transform.position; }
    }
    public float Angle
    {
        get { return transform.rotation.y; }
    }
    public float Speed
    {
        get { return _speed; }
        set { _speed = value; }
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
