using System;
using UnityEngine;

public abstract class Node
{
    Vector2 pos;
    bool walkable;
    bool amrExist;
    NODE_TYPE nodeType;
    public Vector2 Pos { get { return pos; } set { pos = value; } }
    public bool Walkable { get { return walkable; } set { walkable = value; } }
    public bool AmrExist { get { return amrExist; } set { amrExist = value; } }
    public NODE_TYPE NodeType { get { return nodeType; } set { nodeType = value; } }

    public abstract void SetData(Vector2 pos);
}
