using UnityEngine;

public class ActionOrder : MonoBehaviour
{
    #region Attribute
    int _id = 0;
    AIORDER _order = AIORDER.STOP;
    float param = 0f;
    #endregion

    #region Getter & Setter
    public AIORDER Order { get { return _order; }set { _order = value; } }
    public float Param { get { return param; } set { param = value; } }
    #endregion
}
