using UnityEngine;

public class DeviceInfoUI : MonoBehaviour
{
    
    public DeviceListPanel _deviceList;
    public DeviceCountPanel _deviceCount;
    private void AddDevice(StateData data)
    {
        _deviceList.AddPanel(data);
    }

    public void RefreshInfo()
    {
        foreach (var device in Managers.Device.DeviceStates)
        {
            AddDevice(device.Value);
        }

        _deviceCount.RefreshCount();
        _deviceList.RefreshPanel();
    }

    public void OnExitButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
    }
}
