using UnityEngine;

public class ObstacleNode : Node
{
    public override void SetData(Vector2Int pos)
    {
        Walkable = false;
        Pos = pos;
        NodeType = NODE_TYPE.OBSTACLE;
    }
}
