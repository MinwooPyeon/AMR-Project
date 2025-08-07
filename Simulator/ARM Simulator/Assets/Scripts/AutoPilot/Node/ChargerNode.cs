using Unity.VisualScripting;
using UnityEngine;

public class ChargerNode : Node
{
    public override void SetData(Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
        NodeType = NODE_TYPE.CHARGER;
        Parent = null;
    }
}
