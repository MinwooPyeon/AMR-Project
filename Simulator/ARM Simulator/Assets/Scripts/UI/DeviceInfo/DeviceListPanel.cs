using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

public class DeviceListPanel : MonoBehaviour
{
    public GameObject ContentObject;
    public GameObject _devicePanel;

    public List<DeviceInfoPanel> _devicePanels;

    public void AddPanel(StateData data)
    {
        GameObject panel = Instantiate<GameObject>(_devicePanel);
        panel.GetComponent<DeviceInfoPanel>().SerialNumber = data.SerialNumber;
        panel.transform.SetParent(ContentObject.transform, false);
        _devicePanels.Add(panel.GetComponent<DeviceInfoPanel>());
    }

    public void RefreshPanel()
    {
        Debug.Log("Refresh");
        foreach (var devicePanel in _devicePanels)
        {
            devicePanel.SetData();
        }
    }
}
