using UnityEngine;
using UnityEngine.UI;

public class EditButtonAction : MonoBehaviour
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
    public void ChangeEditorFilePath()
    {
        paintCanvas.yamlFileName = yamlPath.text;
        paintCanvas.imageFileName = pngPath.text;
        paintCanvas.saveFileName = savePath.text;
        OffFileSetting();
    }
    public void OpenFileSetting()
    {
        yamlPath.transform.parent.parent.gameObject.SetActive(true);
    }
    public void OffFileSetting()
    {
        yamlPath.transform.parent.parent.gameObject.SetActive(false);
    }
    public void LoadFile()
    {
        paintCanvas.Load();
    }
}