using UnityEngine;

public class MainUIAction : MonoBehaviour
{
    public Canvas VirtualMapCanvas;
    public Canvas PathEditCanvas;
    public Canvas TimeLapseCanvas;
    public Canvas DeviceInfoCanvas;
    public Canvas DeviceSettingCanvas;
    public Canvas MapEditCanvas;

    public MapSpawner mapSpawner;

    public void OnPathEditButtonClicked()
    {
        VirtualMapCanvas.gameObject.SetActive(true);
        PathEditCanvas.gameObject.SetActive(true);
        TimeLapseCanvas.gameObject.SetActive(false);
        DeviceInfoCanvas.gameObject.SetActive(false);
        DeviceSettingCanvas.gameObject.SetActive(false);
        MapEditCanvas.gameObject.SetActive(false);
    }

    public void OnTimelapseButtonClicked()
    {
        VirtualMapCanvas.gameObject.SetActive(true);
        PathEditCanvas.gameObject.SetActive(false);
        TimeLapseCanvas.gameObject.SetActive(true);
        DeviceInfoCanvas.gameObject.SetActive(false);
        DeviceSettingCanvas.gameObject.SetActive(false);
        MapEditCanvas.gameObject.SetActive(false);
    }

    public void OnDeviceInfoButtonClicked()
    {
        VirtualMapCanvas.gameObject.SetActive(false);
        PathEditCanvas.gameObject.SetActive(false);
        TimeLapseCanvas.gameObject.SetActive(false);
        DeviceInfoCanvas.gameObject.SetActive(true);
        DeviceSettingCanvas.gameObject.SetActive(false);
        MapEditCanvas.gameObject.SetActive(false);
    }

    public void OnDeviceSettingButtonClicked()
    {
        VirtualMapCanvas.gameObject.SetActive(true);
        PathEditCanvas.gameObject.SetActive(false);
        TimeLapseCanvas.gameObject.SetActive(false);
        DeviceInfoCanvas.gameObject.SetActive(false);
        DeviceSettingCanvas.gameObject.SetActive(true);
        MapEditCanvas.gameObject.SetActive(false);
    }

    public void OnEditorButtonClicked()
    {
        VirtualMapCanvas.gameObject.SetActive(false);
        PathEditCanvas.gameObject.SetActive(false);
        TimeLapseCanvas.gameObject.SetActive(false);
        DeviceInfoCanvas.gameObject.SetActive(false);
        DeviceSettingCanvas.gameObject.SetActive(false);
        MapEditCanvas.gameObject.SetActive(true);
    }

    public void OnLoadButtonClicked()
    {
        mapSpawner.Load();
    }
}
