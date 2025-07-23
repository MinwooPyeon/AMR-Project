using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ���� ARMs�� StateData�� �����ϰ�,
/// RosBridge�� ���� ����ȭ���ۺ���/������ȭ ó���� ���.
/// </summary>
public class DataManager
{
    [Header("RosBridge ���� (Inspector�� �Ҵ�)")]
    [SerializeField] private RosBridge _rosBridge;

    [Header("�ۺ��� �ֱ� (Hz)")]
    [SerializeField] private float _publishRate = 5f;

    // ID �� StateData ������Ʈ ����
    private readonly Dictionary<int, StateData> _registry = new();

    private float _timer = 0f;

    void Awake()
    {
        if (_rosBridge == null)
        {
            GameObject obj = GameObject.Find("@RosBridge");
            _rosBridge = obj?.GetComponent<RosBridge>();
        }
            
        // RosBridge �̺�Ʈ�� �ݹ� ���
        _rosBridge.OnJsonReceived += HandleIncomingJson;
    }

    void OnDestroy()
    {
        _rosBridge.OnJsonReceived -= HandleIncomingJson;
    }

    void Update()
    {
        // �ֱ������� ��� StateData�� JSON���� ����ȭ���ۺ���
        _timer += Time.deltaTime;
        if (_timer < 1f / _publishRate) return;
        _timer = 0f;

        // 1) DeviceState DTO ����
        var list = new DeviceState[_registry.Count];
        int i = 0;
        foreach (var kv in _registry)
        {
            var sd = kv.Value;
            list[i++] = new DeviceState
            {
                id = kv.Key,
                acceleration = sd.Acceleration,
                chargeAmount = sd.ChargeAmount,
                deviceState = sd.ArmState.ToString(),
                actionState = sd.ActionState.ToString()
            };
        }

        // 2) JSON ����ȭ & �ۺ���
        string json = DeviceStateDataParser.Serialize(list);
        _rosBridge.PublishJson(json);
    }

    /// <summary>ARM �ʱ�ȭ �� StateData ���</summary>
    public void RegisterDevice(int id, StateData data)
    {
        _registry[id] = data;
    }

    /// <summary>ARM �ı� �� ��� ����</summary>
    public void UnregisterDevice(int id)
    {
        _registry.Remove(id);
    }

    // RosBridge�κ��� JSON ���ڿ��� ���� ������ ȣ��˴ϴ�.
    private void HandleIncomingJson(string json)
    {
        var states = DeviceStateDataParser.Deserialize(json);
        foreach (var ds in states)
        {
            if (_registry.TryGetValue(ds.id, out var sd))
            {
                sd.Acceleration = ds.acceleration;
                sd.ChargeAmount = ds.chargeAmount;
                sd.ArmState = Enum.Parse<ARM_STATE>(ds.deviceState);
                sd.ActionState = Enum.Parse<ACTION_STATE>(ds.actionState);
            }
        }
    }
}
