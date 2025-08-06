using UnityEngine;

public class FreeNode : Node
{
    FreeNode(Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
    }
}
