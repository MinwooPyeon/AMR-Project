using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class WalkableMaskBuilder
{
    int _width;
    int _height;
    int[,] _dist;
    int _marginMeter;
    Queue<(int, int)> _que = new();

    public void ChangeWalkableNearWall(Node[,] grid, float resolution)
    {
        _marginMeter = Mathf.CeilToInt(0.3f / resolution);
        _width = grid.GetLength(0);
        _height = grid.GetLength(1);
        _dist = new int[_width, _height];
        _que.Clear();
        ChangeWalkable(grid);
    }

    private void ChangeWalkable(Node[,] grid)
    {
        int INF = 10000000;

        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _dist[x, y] = INF;
                if (grid[x,y] != null && grid[x, y].NodeType == NODE_TYPE.OBSTACLE)
                {
                    _dist[x, y] = 0;
                    _que.Enqueue((x, y));
                }
            }
        }

        var dirs = new (int dx, int dy)[] {
        (1,0), (-1,0), (0,1), (0,-1),
        (1,1), (-1,1), (1,-1), (-1,-1) // 8¹æÇâ
        };

        while (_que.Count > 0)
        {
            var (cx, cy) = _que.Dequeue();
            int cd = _dist[cx, cy];

            if (cd >= _marginMeter) { grid[cx, cy].Walkable = true; continue; }

            foreach(var (dx, dy) in dirs)
            {
                int nx = cx + dx;
                int ny = cy + dy;

                if (grid[nx, ny] == null || ((uint)nx >= (uint)_width || (uint)ny >= (uint)_height)) continue;

                int nd = cd + 1;
                if(nd < _dist[nx, ny])
                {
                    _dist[nx, ny] = nd;
                    grid[nx, ny].Walkable = false;
                    _que.Enqueue((nx, ny));
                }
            }
        }
    }
}
