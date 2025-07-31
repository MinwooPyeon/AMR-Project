using UnityEngine;

public class OccupancyGrid
{
    public int width;
    public int height;
    public float resolution; // meters per cell

    private int[,] grid;

    public OccupancyGrid(int width, int height, float resolution)
    {
        this.width = width;
        this.height = height;
        this.resolution = resolution;
        this.grid = new int[width, height];

        Clear();
    }

    public void Clear()
    {
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = -1; // Unknown
    }

    public void Set(int x, int y, int value)
    {
        if (InBounds(x, y))
            grid[x, y] = value;
    }

    public int Get(int x, int y)
    {
        if (InBounds(x, y))
            return grid[x, y];
        return -1;
    }

    public bool InBounds(int x, int y)
    {
        return (x >= 0 && x < width && y >= 0 && y < height);
    }

    public void AddLidarScan(Vector3 pos, Vector3[] hits)
    {
        foreach(var hit in hits)
        {
            Vector3 worldHit = hit;
            Vector3 local = worldHit - pos;

            int x = Mathf.FloorToInt((local.x + width * resolution / 2) / resolution);
            int y = Mathf.FloorToInt((local.z + height * resolution / 2) / resolution);

            if (InBounds(x, y))
                Set(x, y, 1);
        }
    }

    public Texture2D RenderTexture()
    {
        Texture2D texture = new Texture2D(width, height);
        for(int x = 0; x < width; x++)
        {
            for(int y= 0; y < height; y++)
            {
                int val = grid[x,y];
                Color color = val switch
                {
                    1 => Color.black,
                    0 => Color.white,
                    -1=>Color.gray,
                    _=>Color.magenta
                };
                texture.SetPixel(x, y, color);
            }
        }
        texture.Apply();
        return texture;
    }

    public Vector2Int WorldToGrid(Vector3 worldPos, Vector3 origin)
    {
        Vector3 local = worldPos - origin;
        int x = Mathf.FloorToInt((local.x + width * resolution / 2) / resolution);
        int y = Mathf.FloorToInt((local.z + height * resolution / 2) / resolution);
        return new Vector2Int(x, y);
    }

    public Vector3 GridToWorld(int x, int y, Vector3 origin)
    {
        float wx = (x * resolution) - (width * resolution / 2);
        float wy = (y * resolution) - (height * resolution / 2);
        return origin + new Vector3(wx, 0f, wy);
    }

}
