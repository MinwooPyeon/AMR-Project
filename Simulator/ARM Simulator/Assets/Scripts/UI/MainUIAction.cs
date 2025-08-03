using UnityEngine;

public class MainUIAction : MonoBehaviour
{
    public Canvas VirtualMapCanvas;
    public Canvas PathEditCanvas;
    public Canvas TimeLapseCanvas;
    public Canvas DeviceInfoCanvas;
    public Canvas DeviceSettingCanvas;
    public Canvas MapEditCanvas;
    public Canvas EditorPathEditCanvas;

    public void OnPathEditButtonClicked()
    {
        VirtualMapCanvas.enabled = true;
        PathEditCanvas.enabled = true;
        TimeLapseCanvas.enabled = false;
        DeviceInfoCanvas.enabled = false;
        DeviceSettingCanvas.enabled = false;
        MapEditCanvas.enabled = false;
        EditorPathEditCanvas.enabled = false;
    }

    public void OnTimelapseButtonClicked()
    {
        VirtualMapCanvas.enabled = true;
       PathEditCanvas.enabled = false;
        TimeLapseCanvas.enabled = true;
        DeviceInfoCanvas.enabled = false;
        DeviceSettingCanvas.enabled = false;
        MapEditCanvas.enabled = false;
        EditorPathEditCanvas.enabled = false;
    }

    public void OnDeviceInfoButtonClicked()
    {
        VirtualMapCanvas.enabled = true;
        PathEditCanvas.enabled = false;
        TimeLapseCanvas.enabled = false;
        DeviceInfoCanvas.enabled = true;
        DeviceSettingCanvas.enabled = false;
        MapEditCanvas.enabled = false;
        EditorPathEditCanvas.enabled = false;
    }

    public void OnDeviceSettingButtonClicked()
    {
        VirtualMapCanvas.enabled = false;
        PathEditCanvas.enabled = false;
        TimeLapseCanvas.enabled = false;
        DeviceInfoCanvas.enabled = false;
        DeviceSettingCanvas.enabled = true;
        MapEditCanvas.enabled = false;
        EditorPathEditCanvas.enabled = false;
    }

    public void OnEditorButtonClicked()
    {
        VirtualMapCanvas.enabled = false;
        PathEditCanvas.enabled = false;
        TimeLapseCanvas.enabled = false;
        DeviceInfoCanvas.enabled = false;
        DeviceSettingCanvas.enabled = false;
        MapEditCanvas.enabled = true;
        EditorPathEditCanvas.enabled = true;
    }
}
