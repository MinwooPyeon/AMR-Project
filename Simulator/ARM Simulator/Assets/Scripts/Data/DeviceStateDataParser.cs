// DataParser.cs
using System;
using System.Collections.Generic;
using UnityEngine;

// JSON 파싱용 DTO
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
/// DeviceState[] ↔ JSON 문자열 간 직렬화/역직렬화를 담당합니다.
/// </summary>
public static class DeviceStateDataParser
{
    /// <summary>
    /// DeviceState 배열을 JSON 문자열로 직렬화합니다.
    /// </summary>
    /// <param name="deviceStates">직렬화할 DeviceState 배열</param>
    /// <returns>JSON 문자열</returns>
    public static string Serialize(DeviceState[] deviceStates)
    {
        var wrapper = new DeviceStateList { devices = deviceStates };
        return JsonUtility.ToJson(wrapper);
    }

    /// <summary>
    /// JSON 문자열을 DeviceState 배열로 역직렬화합니다.
    /// </summary>
    /// <param name="json">역직렬화할 JSON 문자열</param>
    /// <returns>DeviceState 배열</returns>
    public static DeviceState[] Deserialize(string json)
    {
        var wrapper = JsonUtility.FromJson<DeviceStateList>(json);
        return wrapper?.devices ?? Array.Empty<DeviceState>();
    }
}
