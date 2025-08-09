using JetBrains.Annotations;
using System;
using UnityEngine;

public abstract class Node
{
    Vector2Int _pos;
    bool _walkable;
    bool _amrExist;
    NODE_TYPE _nodeType;
    Node _parent;
    float _hcost;
    float _gcost;
    public Vector2Int Pos { get { return _pos; } set { _pos = value; } }
    public bool Walkable { get { return _walkable; } set { _walkable = value; } }
    public bool AmrExist { get { return _amrExist; } set { _amrExist = value; } }
    public NODE_TYPE NodeType { get { return _nodeType; } set { _nodeType = value; } }
    public float HCost { get { return _hcost; } set { _hcost = value; } }
    public float GCost { get { return _gcost; } set { _gcost = value; } }
    public float FCost { get { return _hcost + _gcost; } }
    public abstract void SetData(Vector2Int pos);
    public Node Parent {  get { return _parent; } set { _parent = value; } }
    
}
