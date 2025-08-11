using UnityEngine;

public class ObstacleNode : Node
{
    public override void SetData(Vector2 pos)
    {
        Walkable = false;
        Pos = pos;
        NodeType = NODE_TYPE.OBSTACLE;
    }
}
