using System.Collections.Generic;
using UnityEngine;

public static class GridSafeUtil
{
    /// worldPos���� ���� ����� Walkable ���(�׸��� ��/���̸� �ֺ��� Ȯ�� Ž��)
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

    /// center �ֺ� �ݰ� cells ���� ��� Walkable����(���� ���� �Ǵ�)
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

    /// 4-�̿� Walkable ����(����/������ ������)
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

    /// ����� ���� ������ �ְų�(Ŭ�����) �б�(����>=3)�� '���� ����'
    public static bool IsOpenJunction(Node n, float clearanceMeters)
    {
        int cells = Mathf.Max(1, Mathf.CeilToInt(clearanceMeters / Managers.Map.Resolution));
        if (HasClearance(n, cells)) return true;
        return Degree4(n) >= 3;
    }

    /// (�ɼ�) ���� ��θ� �Ųٷ� ��ĵ�ؼ� ���� ����� ���� ���� ã��
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

    /// �� BFS�� ���� ����� '����/���� ����' ã�� (Ż�� ���=steps)
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
