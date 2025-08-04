using UnityEngine;

public class DeviceListPanelUI : MonoBehaviour
{
    public GameObject ContentObject;
    public GameObject _devicePanel;

    public void AddPanel(StateData data)
    {
        GameObject panel = Instantiate<GameObject>(_devicePanel);
        panel.transform.SetParent(ContentObject.transform, false);
    }
}
