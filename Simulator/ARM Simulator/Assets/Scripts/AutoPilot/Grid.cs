using UnityEngine;

public class Grid : MonoBehaviour
{
    Node[,] grid;
    int width, height;

    Grid(int width, int height)
    {
        this.width = width;
        this.height = height;

        grid = new Node[width, height];
    }

    public Node GetLocateNode(int width, int height)
    {
        return grid[width,height];
    }
}
