// DataParser.cs
using System;
using System.Collections.Generic;
using UnityEngine;

// JSON �Ľ̿� DTO
[Serializable]
public class DeviceState
{
    public int id;
    public float acceleration;
    public float chargeAmount;
    public string deviceState;
    public string actionState;
}

public class DeviceStateList
{
    public DeviceState[] devices;
}

/// <summary>
/// DeviceState[] �� JSON ���ڿ� �� ����ȭ/������ȭ�� ����մϴ�.
/// </summary>
public static class DeviceStateDataParser
{
    /// <summary>
    /// DeviceState �迭�� JSON ���ڿ��� ����ȭ�մϴ�.
    /// </summary>
    /// <param name="deviceStates">����ȭ�� DeviceState �迭</param>
    /// <returns>JSON ���ڿ�</returns>
    public static string Serialize(DeviceState[] deviceStates)
    {
        var wrapper = new DeviceStateList { devices = deviceStates };
        return JsonUtility.ToJson(wrapper);
    }

    /// <summary>
    /// JSON ���ڿ��� DeviceState �迭�� ������ȭ�մϴ�.
    /// </summary>
    /// <param name="json">������ȭ�� JSON ���ڿ�</param>
    /// <returns>DeviceState �迭</returns>
    public static DeviceState[] Deserialize(string json)
    {
        var wrapper = JsonUtility.FromJson<DeviceStateList>(json);
        return wrapper?.devices ?? Array.Empty<DeviceState>();
    }
}
