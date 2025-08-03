using UnityEngine;

public struct YamlFile
{
    public float resolution;
    public Vector3 origin;
    public float occThresh, freeThresh;
    
    public Vector2Int[] chargerCells;
    public Vector2Int[] loadCells;
    public Vector2Int[] dropCells;

}

public struct ImageFile
{
    public int width;
    public int height;
    public float[,] probGrid;
    public Texture2D texture;
}