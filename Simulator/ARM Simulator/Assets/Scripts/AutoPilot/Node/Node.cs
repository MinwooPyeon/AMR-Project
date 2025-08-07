using JetBrains.Annotations;
using System;
using UnityEngine;

public abstract class Node
{
    Vector2 _pos;
    bool _walkable;
    bool _amrExist;
    NODE_TYPE _nodeType;
    Node _parent;
    int _hcost;
    public Vector2 Pos { get { return _pos; } set { _pos = value; } }
    public bool Walkable { get { return _walkable; } set { _walkable = value; } }
    public bool AmrExist { get { return _amrExist; } set { _amrExist = value; } }
    public NODE_TYPE NodeType { get { return _nodeType; } set { _nodeType = value; } }
    public int HCost { get { return _hcost; } set { _hcost = value; } }
    public abstract void SetData(Vector2 pos);
    public Node Parent {  get { return _parent; } set { _parent = value; } }
    
}
