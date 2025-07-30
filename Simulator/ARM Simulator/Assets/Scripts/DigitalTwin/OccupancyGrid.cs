using UnityEngine;

public class OccupancyGrid
{
    public int width;
    public int height;
    public float cellSize;
    public Vector3 originPosition;

    private float[,] grid;

    public OccupancyGrid(int width, int height, float cellSize, Vector3 originPosition)
    {
        this.width = width;
        this.height = height;
        this.cellSize = cellSize;
        this.originPosition = originPosition;

        grid = new float[width, height];
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = -1f; // unknown
    }

    public void AddLidarHit(Vector3 worldPos)
    {
        int x = Mathf.FloorToInt((worldPos.x - originPosition.x) / cellSize);
        int y = Mathf.FloorToInt((worldPos.z - originPosition.z) / cellSize);

        if (x >= 0 && y >= 0 && x < width && y < height)
        {
            if (grid[x, y] < 0f) grid[x, y] = 0f; // initialize
            grid[x, y] = Mathf.Min(1.0f, grid[x, y] + 0.1f);
        }
    }

    public float GetProbability(int x, int y)
    {
        if (x < 0 || y < 0 || x >= width || y >= height) return -1;
        return grid[x, y];
    }
}
