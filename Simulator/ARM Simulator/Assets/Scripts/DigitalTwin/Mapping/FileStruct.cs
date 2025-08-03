using UnityEngine;

public struct YamlFile
{
    public float resolution;
    public Vector3 origin;
    public float occThresh, freeThresh;
}

public struct ImageFile
{
    public int width;
    public int height;
    public float[,] probGrid;
    public Texture2D texture;
}