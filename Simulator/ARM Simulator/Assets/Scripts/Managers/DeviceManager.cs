using System.Collections.Generic;
using UnityEngine;

public class DeviceManager
{
    #region Attribute
    Dictionary<string, DeviceController> _devices = new();
    ModuleSyncManager _syncManager;
    int _deviceCount = 0;
    #endregion

    #region Methods
    public Dictionary<string, DeviceController> Devices
    {
        get { return _devices; }
    }
    public int DeviceCount
    {
        get { return _deviceCount; }
        set { _deviceCount = value; }
    }

    public ModuleSyncManager SyncManager { get { return _syncManager; } set { _syncManager = value; } }
    public void DeviceRegister(string id, DeviceController device)
    {
        if (device == null) return;
        if (!_devices.ContainsKey(id)) _devices.Add(id, device);
        else _devices[id] = device;

        Debug.Log(_syncManager);
        _syncManager.RegistModule(id, device.gameObject);
        
    }

    public void DeviceUnregister(string id)
    {
        if (_devices.ContainsKey(id))
        {
            _syncManager.UnregistModule(id);
            _devices.Remove(id);
        }
    }

    public void DeviceActor(string id, ActionOrder order)
    {
        DeviceController device = _devices[id];
        if (device == null) return;

        device.ExcuteOrder(order);
    }

    #endregion
}
