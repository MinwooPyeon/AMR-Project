using UnityEngine;
using UnityEngine.UI;

public class DeviceDetailPanel : MonoBehaviour
{
    public Text timeStampText;
    public Text positionText;
    public Text amrStateText;
    public Text actionStateText;
    public Text issueText;
    public RawImage image;

    public void SetData(SensorFrame frame)
    {
        timeStampText.text = frame.timestamp.ToString("yyyy-MM-dd HH:mm:ss");
        positionText.text = $"X: {frame.position.x}, Y: {frame.position.z}";
        amrStateText.text = frame.amrState.ToString();
        actionStateText.text = frame.actionState.ToString();
        //TODO Error Case
        //TODO Image
    }
}
