using UnityEngine;

public class DeviceInfoUI : MonoBehaviour
{
    
    public DeviceListPanel _deviceList;
    public DeviceCountPanel _deviceCount;
    public void AddDevice(StateData data)
    {
        _deviceList.AddPanel(data);
    }

    public void RefreshInfo()
    {
        _deviceCount.RefreshCount();
    }

    public void OnExitButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
    }
}
