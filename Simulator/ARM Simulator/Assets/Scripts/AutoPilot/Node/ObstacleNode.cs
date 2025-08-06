using UnityEngine;

public class ObstacleNode : Node
{
    ObstacleNode(Vector2 pos)
    {
        Walkable = false;
        Pos = pos;
    }
}
