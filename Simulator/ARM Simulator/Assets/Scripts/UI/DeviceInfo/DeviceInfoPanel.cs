using UnityEngine;
using UnityEngine.UI;

public class DeviceInfoPanel : MonoBehaviour
{
    public Text SerialNumText;
    public Text PositionText;
    public Text StateText;

    private string _serialNumber;
    
    
    public string SerialNumber
    {
        get { return _serialNumber; }
        set { _serialNumber = value; }
    }

    public void SetData()
    {
        SerialNumText.text = _serialNumber;

        StateData data = Managers.Device.DeviceStates[_serialNumber];

        PositionText.text = $"x : {data.GridPosition.x} / y: {data.GridPosition.y}";
        StateText.text = data.AmrState.ToString();
    }

    public void OnParticularButtonClicked()
    {
        GameObject panel = PanelList.Panels.DeviceDetailCanvas;
        
    }
}
