using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DeviceCountPanelUI : MonoBehaviour
{
    public Text totalText;
    public Text chargeText;
    public Text dropText;
    public Text loadText;

    private int _totalCnt = 0;
    private int _chargeCnt = 0;
    private int _dropCnt= 0;
    private int _loadCnt = 0;
    public void RefreshCount()
    {
        ResetCount();
        Dictionary<string, DeviceController> dict = Managers.Device.Devices;
        foreach (var key in dict.Keys)
        {
            StateData data = dict[key].gameObject.GetComponent<StateData>();
            switch (data.ArmState) 
            {
                case AMR_STATE.DROP:
                    _dropCnt++;
                    break;
                case AMR_STATE.LOAD:
                    _loadCnt++;
                    break;
                case AMR_STATE.RECHARGE:
                    _chargeCnt++;
                    break;
            }
            _totalCnt++;
        }

        totalText.text = _totalCnt.ToString();
        chargeText.text = _chargeCnt.ToString();
        dropText.text = _dropCnt.ToString();
        loadText.text = _loadCnt.ToString();
    }

    private void ResetCount()
    {
        _totalCnt = 0;
        _chargeCnt = 0;
        _dropCnt = 0;
        _loadCnt = 0;
    }
}
