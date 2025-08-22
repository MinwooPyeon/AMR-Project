using UnityEngine;

public class CoordinateCalc
{
    public static Vector3 GridToWorld(Vector2Int gridPos, float height, float resolution)
    {
        Vector3 worldPos = new Vector3(gridPos.x * resolution, height, gridPos.y * resolution);
        return worldPos;
    }

    public static Vector2Int WorldToGrid(Vector3 worldPos, float resolution)
    {
        Vector2Int gridPos = new Vector2Int((int)(worldPos.x / resolution), (int)(worldPos.z / resolution));
        return gridPos;
    }
}