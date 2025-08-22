using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

public class FrameListPanel : MonoBehaviour
{
    public GameObject framePanel;
    public GameObject contentObject;

    public void setData(List<SensorFrame> list)
    {
        foreach(SensorFrame frame in list)
        {
            GameObject panel = Instantiate(framePanel, contentObject.transform);
            panel.GetComponent<FramePanel>().SetData(frame);
        }
    }
}
