using UnityEngine;
using UnityEngine.UI;

public class DeviceNumberPanel : MonoBehaviour
{
    public InputField inputField;
    public Slider slider;
    private int _deviceNum;

    public void SetSliderValue()
    {
        slider.value = float.Parse(inputField.text);
        _deviceNum = (int)slider.value;
    }

    public void SetInputValue()
    {
        inputField.text = slider.value.ToString();
        _deviceNum = (int)slider.value;
    }

    public void OnSaveButtonClicked()
    {
        Managers.Device.DeviceCount = _deviceNum;
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
    }
    public void OnCancleButtonClicked()
    {
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
    }


}
