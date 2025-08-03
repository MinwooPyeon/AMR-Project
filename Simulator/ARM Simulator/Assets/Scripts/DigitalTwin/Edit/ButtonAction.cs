using UnityEngine;
using UnityEngine.UI;

public class ButtonAction : MonoBehaviour
{
    public PaintCanvasWithPalette paintCanvas;
    public InputField yamlPath;
    public InputField pngPath;
    public InputField savePath;
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

    public void ChangePath()
    {
        paintCanvas.yamlFileName = yamlPath.text;
        paintCanvas.imageFileName = pngPath.text;
        paintCanvas.saveFileName = savePath.text;
        OffSetting();
    }

    public void OpenSetting()
    {
        yamlPath.transform.parent.parent.gameObject.SetActive(true);
    }

    public void OffSetting()
    {
        yamlPath.transform.parent.parent.gameObject.SetActive(false);
    }
    public void Load()
    {
        paintCanvas.Load();
    }
}