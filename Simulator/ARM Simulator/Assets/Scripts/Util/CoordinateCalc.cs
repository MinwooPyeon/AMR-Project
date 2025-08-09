using UnityEngine;

public class CoordinateCalc
{
    public static Vector3 GridToWorld(Vector2 gridPos, float height, float resolution)
    {
        Vector3 worldPos = new Vector3(gridPos.x * resolution, height, gridPos.y * resolution);
        return worldPos;
    }

    public static Vector2 WorldToGrid(Vector3 worldPos, float resolution)
    {
        Vector2 gridPos = new Vector2((int)(worldPos.x / resolution), (int)(worldPos.z / resolution));
        return gridPos;
    }
}