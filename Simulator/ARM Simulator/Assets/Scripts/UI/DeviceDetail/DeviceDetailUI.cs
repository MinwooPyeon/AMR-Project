using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

public class DeviceDetailUI : MonoBehaviour
{
    public DeviceDetailPanel deviceDetailPanel;
    public FrameListPanel frameListPanel;
    public DevicePanel devicePanel;

    List<SensorFrame> _frames;
    public void SetData(string serialNumber)
    {
        _frames = Managers.Data.GetFrameHistory(serialNumber);
        devicePanel.SetData(serialNumber, _frames[_frames.Count - 1]);
        frameListPanel.setData(_frames);
    }

    public void OnExitButtonClicked()
    {
        PanelList.Panels.DeviceDetailCanvas.SetActive(false);
    }
}
