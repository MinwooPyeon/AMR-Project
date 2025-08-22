using UnityEngine;
using UnityEngine.UI;

public class DevicePanel : MonoBehaviour
{
    public Text serialNumberText;
    public Text AMRStateText;
    public Text actionState;
    public Text issueCntText;

    public void SetData(string serialNumber, SensorFrame frame)
    {
        serialNumberText.text = serialNumber;
        AMRStateText.text = frame.amrState.ToString();
        actionState.text = frame.actionState.ToString();
    }
}
