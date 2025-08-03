
using UnityEngine;

public class ButtonAction : MonoBehaviour
{
    public PaintCanvasWithPalette paintCanvas;
    public void ChangeColorWhite()
    {
        paintCanvas.ChangeColor(BrushColor.White);
    }

    public void ChangeColorBlack()
    {
        paintCanvas.ChangeColor(BrushColor.Black);
    }

    public void ChangeColorRed()
    {
        paintCanvas.ChangeColor(BrushColor.Red);
    }

    public void ChangeColorGreen()
    {
       paintCanvas.ChangeColor(BrushColor.Green);
    }

    public void ChangeColorBlue()
    {
        paintCanvas.ChangeColor(BrushColor.Blue);
    }

    public void SaveMap()
    {
        paintCanvas.SaveCanvas("map.png");
    }
}