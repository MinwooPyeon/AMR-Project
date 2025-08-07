using UnityEngine;

public class FreeNode : Node
{
    public override void SetData(Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
        NodeType = NODE_TYPE.FREE;
        Parent = null;
    }
}
