using UnityEngine;
using UnityEngine.UI;

public class PathEditButtonAction : MonoBehaviour
{
    public MapSpawner mapSpawner;
    public InputField yamlPath;
    public InputField pngPath;

    public void OnSaveButtonClicked()
    {
        mapSpawner.yamlFileName = yamlPath.text;
        mapSpawner.imageFileName = pngPath.text;
        PanelList.Panels.PathEditCanvas.SetActive(false);
    }

    public void OnCancleButtonClicked()
    {
        PanelList.Panels.PathEditCanvas.SetActive(false);
    }
}
