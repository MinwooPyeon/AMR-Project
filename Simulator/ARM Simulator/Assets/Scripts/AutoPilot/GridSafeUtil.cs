using System.Collections.Generic;
using UnityEngine;

public static class GridSafeUtil
{
    /// worldPos에서 가장 가까운 Walkable 노드(그리드 밖/벽이면 주변을 확장 탐색)
    public static bool TryGetNearestWalkableNode(Vector3 worldPos, out Node node, int maxRing = 25)
    {
        var grid = Managers.Map.Grid;
        var gridA = grid.grid;
        node = grid.GetLocateNode(worldPos);
        if (node != null && node.Walkable) return true;

        Vector2Int p = CoordinateCalc.WorldToGrid(worldPos, Managers.Map.Resolution);
        for (int r = 1; r <= maxRing; r++)
        {
            for (int dy = -r; dy <= r; dy++)
            {
                int dx = r - Mathf.Abs(dy);
                if (Try(p.x + dx, p.y + dy, out node)) return true;
                if (dx != 0 && Try(p.x - dx, p.y + dy, out node)) return true;
            }
        }
        node = null; return false;

        bool Try(int x, int y, out Node n)
        {
            n = null;
            Vector2Int q = new Vector2Int(x, y);
            if (!grid.InBounds(q.x , q.y)) return false;
            n = gridA[q.x , q.y];
            return n != null && n.Walkable;
        }
    }

    /// center 주변 반경 cells 내가 모두 Walkable인지(여유 공간 판단)
    public static bool HasClearance(Node center, int cells)
    {
        var grid = Managers.Map.Grid;
        var gridA = grid.grid;
        Vector2Int c = center.Pos;
        for (int dy = -cells; dy <= cells; dy++)
            for (int dx = -cells; dx <= cells; dx++)
            {
                if (dx * dx + dy * dy > cells * cells) continue;
                Vector2Int p = new Vector2Int(c.x + dx, c.y + dy);
                if (!grid.InBounds(p.x , p.y)) return false;
                var n = gridA[p.x, p.y];
                if (n == null || !n.Walkable) return false;
            }
        return true;
    }

    /// 4-이웃 Walkable 개수(교차/갈림길 판정용)
    public static int Degree4(Node n)
    {
        var grid = Managers.Map.Grid;
        var gridA = grid.grid;
        Vector2Int p = n.Pos;
        int deg = 0;
        var dirs = new[] { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };
        foreach (var d in dirs)
        {
            Vector2Int q = p + d;
            if (!grid.InBounds(q.x, q.y)) continue;
            var m = gridA[q.x , q.y];
            if (m != null && m.Walkable) deg++;
        }
        return deg;
    }

    /// 충분한 여유 공간이 있거나(클리어런스) 분기(차수>=3)면 '오픈 지점'
    public static bool IsOpenJunction(Node n, float clearanceMeters)
    {
        int cells = Mathf.Max(1, Mathf.CeilToInt(clearanceMeters / Managers.Map.Resolution));
        if (HasClearance(n, cells)) return true;
        return Degree4(n) >= 3;
    }

    /// (옵션) 현재 경로를 거꾸로 스캔해서 가장 가까운 오픈 지점 찾기
    public static bool TryFindOpenAlongRouteBackwards(List<Node> route, int currentIndex, float clearanceMeters, int maxBackSteps, out Node open)
    {
        open = null;
        if (route == null || route.Count == 0) return false;

        for (int i = Mathf.Clamp(currentIndex - 1, 0, route.Count - 1), steps = 0;
             i >= 0 && steps <= maxBackSteps; i--, steps++)
        {
            var n = route[i];
            if (n != null && n.Walkable && IsOpenJunction(n, clearanceMeters))
            {
                open = n; return true;
            }
        }
        return false;
    }

    /// ★ BFS로 가장 가까운 '오픈/교차 지점' 찾기 (탈출 비용=steps)
    public static bool TryFindNearestOpenBFS(Node start, float clearanceMeters, int maxSteps, out Node open, out int steps)
    {
        open = null; steps = 0;
        if (start == null || !start.Walkable) return false;

        var grid = Managers.Map.Grid;
        var gridA = grid.grid;
        var q = new Queue<Node>();
        var visited = new HashSet<Node>();
        q.Enqueue(start); visited.Add(start);
        int depth = 0;

        while (q.Count > 0 && depth <= maxSteps)
        {
            int cnt = q.Count;
            while (cnt-- > 0)
            {
                var n = q.Dequeue();
                if (IsOpenJunction(n, clearanceMeters))
                {
                    open = n; steps = depth; return true;
                }

                Vector2Int p = n.Pos;
                var dirs = new[] { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };
                foreach (var d in dirs)
                {
                    Vector2Int np = p + d;
                    if (!grid.InBounds(np.x , np.y)) continue;
                    var m = gridA[np.x, np.y];
                    if (m == null || !m.Walkable || visited.Contains(m)) continue;
                    visited.Add(m); q.Enqueue(m);
                }
            }
            depth++;
        }
        return false;
    }
}
