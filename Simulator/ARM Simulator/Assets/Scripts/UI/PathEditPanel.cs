using UnityEngine;
using UnityEngine.UI;

public class PathEditButtonAction : MonoBehaviour
{
    public MapSpawner mapSpawner;
    public InputField yamlPath;
    public InputField pngPath;
    public Canvas pathEditCanvas;

    public void OnSaveButtonClicked()
    {
        mapSpawner.yamlFileName = yamlPath.text;
        mapSpawner.imageFileName = pngPath.text;
        pathEditCanvas.gameObject.SetActive(false);
    }

    public void OnCancleButtonClicked()
    {
        pathEditCanvas.gameObject.SetActive(false);
    }
}
