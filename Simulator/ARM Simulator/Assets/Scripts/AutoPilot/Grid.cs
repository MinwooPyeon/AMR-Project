using System;
using System.Collections.Generic;
using UnityEngine;

public class Grid
{

    readonly Dictionary<NODE_TYPE, Func<Node>> dict = new()
    {
        {NODE_TYPE.FREE, ()=>new FreeNode() },
        {NODE_TYPE.OBSTACLE, ()=>new ObstacleNode() },
        {NODE_TYPE.CHARGER, ()=>new ChargerNode() },
        {NODE_TYPE.LOADER, ()=>new LoaderNode() },
        {NODE_TYPE.DROPER, ()=>new DroperNode() },
    };
    Node[,] grid;
    int _width, _height;

    public void SetSize(int width, int height)
    {
        this._width = width;
        this._height = height;

        grid = new Node[width, height];
    }

    public Node GetLocateNode(int width, int height)
    {
        return grid[width,height];
    }

    public void SetNode(Vector3 worldPos, NODE_TYPE type)
    {
        Node node = dict[type]();
        node.SetData(new Vector2(worldPos.x, worldPos.z));
        grid[(int)worldPos.x, (int)worldPos.z] = node;

    }
}
