using UnityEngine;

public class DeviceInfoUI : MonoBehaviour
{
    
    public DeviceListPanel _deviceList;
    public DeviceCountPanel _deviceCount;
<<<<<<< HEAD
    public void AddDevice(StateData data)
=======
    private void AddDevice(StateData data)
>>>>>>> origin/develop
    {
        _deviceList.AddPanel(data);
    }

    public void RefreshInfo()
    {
<<<<<<< HEAD
        _deviceCount.RefreshCount();
=======
        foreach (var device in Managers.Device.DeviceStates)
        {
            AddDevice(device.Value);
        }

        _deviceCount.RefreshCount();
        _deviceList.RefreshPanel();
>>>>>>> origin/develop
    }

    public void OnExitButtonClicked()
    {
        PanelList.Panels.VirtualMapCanvas.SetActive(true);
        PanelList.Panels.DeviceInfoCanvas.SetActive(false);
    }
}
