using UnityEngine;

public class Node
{
    Vector2 pos;
    bool walkable;
    bool amrExist;

    public Vector2 Pos { get { return pos; } set { pos = value; } }
    public bool Walkable { get { return walkable; } set { walkable = value; } }
    public bool AmrExist { get { return amrExist; } set { amrExist = value; } }
}
