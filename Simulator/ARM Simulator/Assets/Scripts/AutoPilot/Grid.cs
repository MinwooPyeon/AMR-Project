using System;
using System.Collections.Generic;
using UnityEngine;

public class Grid
{
    private readonly int[] _dirX = { -1, 0, 0, 1 };
    private readonly int[] _dirY = { 0, -1, 1, 0 };

    private readonly Dictionary<NODE_TYPE, Func<Node>> dict = new()
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

    public List<Node> GetNeighborNode(Node node)
    {
        List<Node> neighbor = new();

        Vector2 pos = node.Pos;

        for(int i = 0; i < 4; i++)
        {
            int dx = (int)pos.x + _dirX[i];
            int dy = (int)pos.y + _dirY[i];

            if (CheckBound(dx, dy) && grid[dx, dy].Walkable)
            {
                //Debug.Log(grid[dx, dy]);
                neighbor.Add(grid[dx, dy]);
            }
        }
        return neighbor;
    }

    private bool CheckBound(int x,  int y)
    {
        if (x < 0 || x >= _width || y < 0 || y >= _height) return false;
        return true;
    }
}

