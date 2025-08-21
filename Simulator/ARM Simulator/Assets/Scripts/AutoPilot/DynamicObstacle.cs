using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// A* 그리드에 원형 영역을 임시로 Walkable=false로 표시하고 일정 시간 뒤 복구
public static class DynamicObstacle
{
    public static MonoBehaviour CoroutineHost; // Awake/Start에서 한 번만 지정

    public static void MarkCircleTemporary(Vector3 worldCenter, float radius, float durationSec)
    {
        var grid = Managers.Map.Grid;
        var gridA = grid.grid;
        Vector2Int center = CoordinateCalc.WorldToGrid(worldCenter, Managers.Map.Resolution);
        int cells = Mathf.CeilToInt(radius / Managers.Map.Resolution);

        List<Node> changed = new List<Node>(cells * cells * 4);
        for (int dy = -cells; dy <= cells; dy++)
            for (int dx = -cells; dx <= cells; dx++)
            {
                if (dx * dx + dy * dy > cells * cells) continue;
                Vector2Int p = new Vector2Int(center.x + dx, center.y + dy);
                if (!grid.InBounds(p.x, p.y)) continue;

                Node n = gridA[p.x, p.y];
                if (n!= null && n.Walkable)
                {
                    n.Walkable = false;
                    changed.Add(n);
                }
            }

        if (CoroutineHost != null)
            CoroutineHost.StartCoroutine(RestoreAfterDelay(changed, durationSec));
        else
            Debug.LogWarning("[DynamicObstacle] CoroutineHost 미지정: 자동 복구 불가");
    }

    private static IEnumerator RestoreAfterDelay(List<Node> changed, float delay)
    {
        yield return new WaitForSeconds(delay);
        foreach (var n in changed) n.Walkable = true;
    }
}

/// (선택) 두 지점 사이를 일정 간격으로 원형 차단하여 '선분 차단' 효과
public static class DynamicObstacleEx
{
    public static void MarkSegmentTemporary(Vector3 a, Vector3 b, float radius, float durationSec)
    {
        float cell = Mathf.Max(Managers.Map.Resolution * 0.8f, 0.01f);
        int steps = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(a, b) / cell));
        for (int i = 0; i <= steps; i++)
        {
            Vector3 p = Vector3.Lerp(a, b, i / (float)steps);
            DynamicObstacle.MarkCircleTemporary(p, radius, durationSec);
        }
    }
}
