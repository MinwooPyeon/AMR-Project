using Unity.VisualScripting;
using UnityEngine;

public class DroperNode : Node
{
    public override void SetData(Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
        NodeType = NODE_TYPE.DROPER;
    }
}
