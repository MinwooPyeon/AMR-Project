using UnityEngine;
using UnityEngine.UI;

public class EditButtonAction : MonoBehaviour
{
    public PaintCanvasWithPalette paintCanvas;
<<<<<<< HEAD

=======
    
>>>>>>> origin/develop
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
        PanelList.Panels.EditorPathEditCanvas.SetActive(true);
    }
    public void OffFileSetting()
    {
        PanelList.Panels.EditorPathEditCanvas.SetActive(false);
    }
    public void LoadFile()
    {
        paintCanvas.Load();
    }
}