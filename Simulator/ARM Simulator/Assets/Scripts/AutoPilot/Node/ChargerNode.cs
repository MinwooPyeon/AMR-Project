using Unity.VisualScripting;
using UnityEngine;

public class ChargerNode : Node
{
    ChargerNode (Vector2 pos)
    {
        Walkable = true;
        Pos = pos;
    }
}
