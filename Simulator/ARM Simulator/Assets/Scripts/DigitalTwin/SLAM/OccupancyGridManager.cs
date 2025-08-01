// OccupancyGridManager.cs (No Tilemap Version)
using UnityEngine;
using System;
using UnityEditor;

public class OccupancyGridManager : MonoBehaviour
{
    public int width = 200; // grid width in cells
    public int height = 200; // grid height in cells
    public float resolution = 0.1f; // cell size in world units
    public Vector3 origin = Vector3.zero; // center of grid in world space
    public float maxLidarRange = 9f;

    private int[,] grid;
    private Texture2D mapTexture;
    private GameObject displayPlane;

    public static OccupancyGridManager Instance { get; private set; }

    private void Awake()
    {
        if (Instance != null) Destroy(gameObject);
        Instance = this;

        grid = new int[width, height];
        mapTexture = new Texture2D(width, height);

        Clear();

        // Create display plane
        displayPlane = GameObject.CreatePrimitive(PrimitiveType.Quad);
        displayPlane.name = "OccupancyMap";
        displayPlane.transform.localScale = new Vector3(width * resolution, height * resolution, 1);
        displayPlane.transform.position = origin + new Vector3(0, 2f, 0); // slightly above ground
        displayPlane.transform.rotation = Quaternion.Euler(90, 0, 0); // face upward
        displayPlane.GetComponent<Renderer>().material = new Material(Shader.Find("Unlit/Texture"));
        displayPlane.GetComponent<Renderer>().material.mainTexture = mapTexture;
    }

    public void Clear()
    {
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = -1; // -1: unknown, 0: free, 1: occupied

        UpdateTexture();
    }
    public void AddLidarPoints(Vector3[] points, LidarSensor sensor)
    {
        Debug.Log("[GridManager] Adding Lidar Points: " + points.Length + " points.");
        Vector3 start = sensor.gameObject.transform.parent.transform.position;
        

        foreach (var world in points)
        {
            float dist = Vector3.Distance(start, world);
            
            bool actuallyHit = dist < maxLidarRange;
            Debug.Log(dist);
            AddLidarPath(start, world, maxLidarRange, actuallyHit);
        }
        UpdateTexture();
    }
    public void AddLidarPath(Vector3 start, Vector3 hit, float maxDistance, bool actuallyHit)
    {
        Vector3 localStart = start - origin;
        Vector3 localHit = hit - origin;

        int x0 = Mathf.FloorToInt((localStart.x + width * resolution / 2) / resolution);
        int y0 = Mathf.FloorToInt((localStart.z + height * resolution / 2) / resolution);
        int x1 = Mathf.FloorToInt((localHit.x + width * resolution / 2) / resolution);
        int y1 = Mathf.FloorToInt((localHit.z + height * resolution / 2) / resolution);

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;
        int maxIteration = width + height;

        for (int i = 0; i < maxIteration; i++)
        {
            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                if (x == x1 && y == y1)
                {
                    if (actuallyHit && InBounds(x, y))
                        grid[x, y] = 1;
                    break;
                }
                else
                    grid[x, y] = 0;
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    }

    private bool InBounds(int x, int y)
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    private void UpdateTexture()
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                Color color = Color.black;
                switch (grid[x, y])
                {
                    case -1: color = Color.gray; break;
                    case 0: color = Color.white; break;
                    case 1: color = Color.black; break;
                }
                mapTexture.SetPixel(x, y, color);
            }
        }
        mapTexture.Apply();
    }

    public void UpdateAMRPosition(Vector3 worldPos)
    {
        Vector3 local = worldPos - origin;
        int x = Mathf.FloorToInt((local.x + width * resolution / 2) / resolution);
        int y = Mathf.FloorToInt((local.z + height * resolution / 2) / resolution);

        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            mapTexture.SetPixel(x, y, Color.red);
            mapTexture.Apply();
        }
    }

    public void SaveMap(string path)
    {
        string serialized = JsonUtility.ToJson(new OccupancyGridData(grid));
        System.IO.File.WriteAllText(path, serialized);
    }

    public void LoadMap(string path)
    {
        if (!System.IO.File.Exists(path)) return;
        string json = System.IO.File.ReadAllText(path);
        var data = JsonUtility.FromJson<OccupancyGridData>(json);
        grid = data.ToArray();
        UpdateTexture();
    }

    [Serializable]
    public class OccupancyGridData
    {
        public int width;
        public int height;
        public int[] flatGrid;

        public OccupancyGridData(int[,] grid)
        {
            width = grid.GetLength(0);
            height = grid.GetLength(1);
            flatGrid = new int[width * height];

            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    flatGrid[y * width + x] = grid[x, y];
        }

        public int[,] ToArray()
        {
            int[,] result = new int[width, height];
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    result[x, y] = flatGrid[y * width + x];
            return result;
        }
    }
}
