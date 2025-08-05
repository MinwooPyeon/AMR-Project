using UnityEngine;
using UnityEngine.UI;

public class TimelapsePanel : MonoBehaviour
{
    public InputField inputField;
    public Slider slider;

    private float _timeScale;

    public void SetSliderValue()
    {
        slider.value = float.Parse(inputField.text);
        _timeScale = slider.value;
    }

    public void SetInputValue()
    {
        inputField.text = slider.value.ToString();
        _timeScale = slider.value;
    }

    public void OnSaveButtonClicked()
    {
        ChangeTImeScale();
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
    }
    public void OnCancleButtonClicked()
    {
        PanelList.Panels.TimeLapseCanvas.SetActive(false);
    }
    private void ChangeTImeScale()
    {
        Time.timeScale = _timeScale;
    }


}
