using System.Collections.Generic;
using UnityEngine;

public class DeviceManager
{
    #region Attribute
    Dictionary<int, DeviceController> _devices = new();
    #endregion

    #region Methods
    public void DeviceRegister(int id, DeviceController device)
    {
        if (device == null) return;
        if (!_devices.ContainsKey(id)) _devices.Add(id, device);
        else _devices[id] = device;
    }

    public void DeviceUnregister(int id)
    {
        if (_devices.ContainsKey(id))
            _devices.Remove(id);
    }

    public void DeviceActor(int id, ActionOrder order)
    {
        DeviceController device = _devices[id];
        if (device == null) return;

        device.ExcuteOrder(order);
    }
    #endregion
}
