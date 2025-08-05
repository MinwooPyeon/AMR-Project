using UnityEngine;

public class MainUIAction : MonoBehaviour
{
    public MapSpawner mapSpawner;
    public AMRSpawner amrSpawner;
    public void OnPathEditButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        
        PanelList.Panels.PathEditCanvas.SetActive(true);
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
        PanelList.Panels.DeviceNumberCanvas.SetActive(false);
        PanelList.Panels.MapEditCanvas.SetActive(false);
    }

    public void OnTimelapseButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        PanelList.Panels.PathEditCanvas.SetActive(false);
        PanelList.Panels.TimeLapseCanvas.SetActive(true);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
        PanelList.Panels.DeviceNumberCanvas.SetActive(false);
        PanelList.Panels.MapEditCanvas.SetActive(false);
    }

    public void OnDeviceInfoButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(false);
        PanelList.Panels.PathEditCanvas.SetActive(false);
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
        PanelList.Panels.DeviceInfoCanvas.SetActive(true);
        PanelList.Panels.DeviceNumberCanvas.SetActive(false);
        PanelList.Panels.MapEditCanvas.SetActive(false);
    }

    public void OnDeviceSettingButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        PanelList.Panels.PathEditCanvas.SetActive(false);
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
        PanelList.Panels.DeviceNumberCanvas.SetActive(true);
        PanelList.Panels.MapEditCanvas.SetActive(false);
    }

    public void OnEditorButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(false);
        PanelList.Panels.PathEditCanvas.SetActive(false);
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
        PanelList.Panels.DeviceNumberCanvas.SetActive(false);
        PanelList.Panels.MapEditCanvas.SetActive(true);
    }

    public void OnLoadButtonClicked()
    {
        mapSpawner.Load();
        amrSpawner.OnLoad(PanelList.Panels.DeviceNumberCanvas.GetComponent<DeviceNumberPanel>().GetDeviceNumber());
        Managers.OnStart();
    }
}
