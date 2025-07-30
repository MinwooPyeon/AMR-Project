using UnityEngine;
using UnityEngine.Tilemaps;

public class SLAMTilemapBuilder : MonoBehaviour
{
    public Tilemap tilemap;
    public TileBase occupiedTile;
    public TileBase freeTile;

    public int gridWidth = 100;
    public int gridHeight = 100;
    public float cellSize = 0.5f;
    public Vector3 originPosition = Vector3.zero;

    public GameObject AMR;
    private OccupancyGrid occupancyGrid;

    void Awake()
    {
        occupancyGrid = new OccupancyGrid(gridWidth, gridHeight, cellSize, originPosition);
    }

    private void Update()
    {
        UpdateLidar(Managers.Data.GetLatestFrame(AMR.GetInstanceID()).lidarPoints);
    }

    public void UpdateLidar(Vector3[] pointCloud)
    {
        foreach (var point in pointCloud)
        {
            occupancyGrid.AddLidarHit(point);
        }
        RenderTilemap();
    }

    void RenderTilemap()
    {
        tilemap.ClearAllTiles();
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                float prob = occupancyGrid.GetProbability(x, y);

                if (prob >= 0.6f)
                    tilemap.SetTile(new Vector3Int(x, y, 0), occupiedTile);
                else if (prob >= 0f && prob < 0.3f)
                    tilemap.SetTile(new Vector3Int(x, y, 0), freeTile);
                // else unknown, skip drawing
            }
        }
    }
}