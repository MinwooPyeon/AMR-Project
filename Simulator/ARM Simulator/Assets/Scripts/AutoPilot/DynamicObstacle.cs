using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// A* �׸��忡 ���� ������ �ӽ÷� Walkable=false�� ǥ���ϰ� ���� �ð� �� ����
public static class DynamicObstacle
{
    public static MonoBehaviour CoroutineHost; // Awake/Start���� �� ���� ����

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
            Debug.LogWarning("[DynamicObstacle] CoroutineHost ������: �ڵ� ���� �Ұ�");
    }

    private static IEnumerator RestoreAfterDelay(List<Node> changed, float delay)
    {
        yield return new WaitForSeconds(delay);
        foreach (var n in changed) n.Walkable = true;
    }
}

/// (����) �� ���� ���̸� ���� �������� ���� �����Ͽ� '���� ����' ȿ��
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
