using UnityEngine;
using UnityEngine.UI;

public class FramePanel : MonoBehaviour
{
    public Text timeStampText;
    public Text issueText;
    public SensorFrame frame;

    public void SetData(SensorFrame frame)
    {
        this.frame = frame;
        timeStampText.text = frame.timestamp.ToString("yyyy-MM-dd HH:mm:ss");
        //issueText.text = frame.issueCase.ToString();
    }

    public void OnClicked()
    {
        PanelList.Panels.DeviceDetailCanvas.GetComponent<DeviceDetailPanel>().SetData(frame);
    }
}
