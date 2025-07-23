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

        Managers.Data.RegisterDevice(id, device.gameObject.GetComponent<StateData>());
    }

    public void DeviceUnregister(int id)
    {
        if (_devices.ContainsKey(id))
        {
            Managers.Data.UnregisterDevice(id);
            _devices.Remove(id);
        }
    }

    public void DeviceActor(int id, ActionOrder order)
    {
        DeviceController device = _devices[id];
        if (device == null) return;

        device.ExcuteOrder(order);
    }
    #endregion
}
