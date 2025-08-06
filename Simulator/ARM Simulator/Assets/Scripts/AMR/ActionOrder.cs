using UnityEngine;

public class ActionOrder : MonoBehaviour
{
    #region Attribute
    ACTION_STATE _order = ACTION_STATE.STOP;
    float _param = 10f;
    #endregion

    #region Getter & Setter
    public int Id { get { return _id; } set { _id = value; } }
    public ACTION_STATE Order { get { return _order; }set { _order = value; } }
    public float Param { get { return _param; } set { _param = value; } }
    #endregion
}
