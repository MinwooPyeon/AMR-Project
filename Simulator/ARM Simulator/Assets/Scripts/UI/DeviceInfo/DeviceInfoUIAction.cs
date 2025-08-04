using UnityEngine;

public class DeviceInfoUIAction : MonoBehaviour
{
    
    public DeviceListPanelUI _deviceList;
    public DeviceCountPanelUI _deviceCount;
    public void AddDevice(StateData data)
    {
        _deviceList.AddPanel(data);
        
    }

    public void RefreshInfo()
    {
        _deviceCount.RefreshCount();
    }
}
