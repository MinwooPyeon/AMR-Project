using Unity.VisualScripting;
using UnityEngine;

public class LoaderNode : Node
{
    LoaderNode (Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
    }
}
