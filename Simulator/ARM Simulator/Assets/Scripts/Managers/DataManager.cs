using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 여러 ARMs의 StateData를 관리하고,
/// RosBridge를 통해 직렬화·퍼블리시/역직렬화 처리만 담당.
/// </summary>
public class DataManager
{
    [Header("RosBridge 참조 (Inspector에 할당)")]
    [SerializeField] private RosBridge _rosBridge;

    [Header("퍼블리시 주기 (Hz)")]
    [SerializeField] private float _publishRate = 5f;

    // ID → StateData 컴포넌트 매핑
    private readonly Dictionary<int, StateData> _registry = new();

    private float _timer = 0f;

    void Awake()
    {
        if (_rosBridge == null)
        {
            GameObject obj = GameObject.Find("@RosBridge");
            _rosBridge = obj?.GetComponent<RosBridge>();
        }
            
        // RosBridge 이벤트에 콜백 등록
        _rosBridge.OnJsonReceived += HandleIncomingJson;
    }

    void OnDestroy()
    {
        _rosBridge.OnJsonReceived -= HandleIncomingJson;
    }

    void Update()
    {
        // 주기적으로 모든 StateData를 JSON으로 직렬화·퍼블리시
        _timer += Time.deltaTime;
        if (_timer < 1f / _publishRate) return;
        _timer = 0f;

        // 1) DeviceState DTO 생성
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

        // 2) JSON 직렬화 & 퍼블리시
        string json = DeviceStateDataParser.Serialize(list);
        _rosBridge.PublishJson(json);
    }

    /// <summary>ARM 초기화 시 StateData 등록</summary>
    public void RegisterDevice(int id, StateData data)
    {
        _registry[id] = data;
    }

    /// <summary>ARM 파괴 시 등록 해제</summary>
    public void UnregisterDevice(int id)
    {
        _registry.Remove(id);
    }

    // RosBridge로부터 JSON 문자열이 들어올 때마다 호출됩니다.
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
