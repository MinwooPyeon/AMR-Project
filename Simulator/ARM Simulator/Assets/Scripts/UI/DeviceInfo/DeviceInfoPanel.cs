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

<<<<<<< HEAD
        StateData data = Managers.Device.Devices[_serialNumber].GetComponent<StateData>();
=======
        StateData data = Managers.Device.DeviceStates[_serialNumber];
>>>>>>> origin/develop

        PositionText.text = $"x : {data.Position.x} / y: {data.Position.y}";
        StateText.text = data.AmrState.ToString();
    }

    public void OnParticularButtonClicked()
    {

    }
}
