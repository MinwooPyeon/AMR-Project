using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeviceManager
{
    #region Attribute
    Dictionary<string, DeviceController> _devices = new();
    Dictionary<string, VirtualDeviceController> _virtualDevices = new();
    Dictionary<string, StateData> _deviceState = new();
    ModuleSyncManager _syncManager;
    MqttPublisher _mqttPublisher;
    int _deviceCount = 0;
    #endregion

    #region Methods
    public Dictionary<string, DeviceController> Devices
    {
        get { return _devices; }
    }
    public Dictionary<string, VirtualDeviceController> VirtualDevices
    {
        get { return _virtualDevices; }
    }
    public Dictionary<string, StateData> DeviceStates
    {
        get { return _deviceState; }
    }
    public int DeviceCount
    {
        get { return _deviceCount; }
        set { _deviceCount = value; }
    }
    public ModuleSyncManager SyncManager { get { return _syncManager; } set { _syncManager = value; } }


    public void RegistVirtualDevice(VirtualDeviceController device)
    {
        if (device == null) return;
        string serial = _deviceCount++.ToString();
        if (!_virtualDevices.ContainsKey(serial)) _virtualDevices.Add(serial, device);
        else _virtualDevices[serial] = device;

        StateData state = device.gameObject.GetComponent<StateData>();
        if (!_deviceState.ContainsKey(serial)) _deviceState.Add(serial, state);
        else _deviceState[serial] = state;
        state.SerialNumber = serial;
        _syncManager.RegistModule(serial, device.gameObject);
    }

    public void DeviceUnregister(string id)
    {
        if (_devices.ContainsKey(id))
        {
            _devices.Remove(id);
            _deviceState.Remove(id);
        }
    }

    public void VirtualDeviceUnregister(string id)
    {
        if (_virtualDevices.ContainsKey(id))
        {
            _syncManager.UnregistModule(id);
            _virtualDevices.Remove(id);
            _deviceState.Remove(id);
        }
    }


    public void DeviceActor(string id, ActionOrder order)
    {
        DeviceController device = _devices[id];
        if (device == null) return;

        device.ExcuteOrder(order);
    }

    public void RegistRealDevice(StatusMsg msg)
    {
        if (_devices[msg.serialNumber] != null) return;

        GameObject obj = Managers.Resource.Instantiate("Prefab/Device/AMR_Real_Device");
        obj.transform.position = new Vector3(msg.position.x, 0, msg.position.y);
        _devices.Add(msg.serialNumber, obj.GetComponent<DeviceController>());
        _deviceState.Add(msg.serialNumber, obj.GetComponent<StateData>());
    }
    
    #endregion
}
