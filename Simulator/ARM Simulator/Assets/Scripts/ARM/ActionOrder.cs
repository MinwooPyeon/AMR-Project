using UnityEngine;

public class ActionOrder : MonoBehaviour
{
    #region Attribute
    int _id = 0;
    ACTION_STATE _order = ACTION_STATE.STOP;
    float param = 0f;
    #endregion

    #region Getter & Setter
    public ACTION_STATE Order { get { return _order; }set { _order = value; } }
    public float Param { get { return param; } set { param = value; } }
    #endregion
}
