using Unity.VisualScripting;
using UnityEngine;

public class ChargerNode : Node
{
    public override void SetData(Vector2Int pos)
    {
        Walkable = true;
        Pos = pos;
        NodeType = NODE_TYPE.CHARGER;
        Parent = null;
    }
}
