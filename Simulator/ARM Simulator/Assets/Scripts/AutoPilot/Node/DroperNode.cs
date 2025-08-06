using Unity.VisualScripting;
using UnityEngine;

public class DroperNode : Node
{
    DroperNode (Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
    }
}
