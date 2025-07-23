using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using StringMsg = RosMessageTypes.Std.StringMsg;
using System;

public class RosBridge : MonoBehaviour
{
    [Header("ROS 연결 설정")]
    [SerializeField] string _pubTopic = "unity/server";
    [SerializeField] string _subTopic = "server/unity";

    ROSConnection _ros;
    float timer = 0f;

    /// <summary>JSON 문자열 수신 시 호출할 이벤트</summary>
    public event Action<string> OnJsonReceived;

    void Awake()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<StringMsg>(_pubTopic);
        _ros.Subscribe<StringMsg>(_subTopic, OnReceiveMsg);
    }

    void OnReceiveMsg(StringMsg msg)
    {
        Debug.Log($"[Server → Unity] Received JSON: {msg.data}");
        OnJsonReceived?.Invoke(msg.data);
    }

    public void PublishJson(string json)
    {
        _ros.Publish(_pubTopic, new StringMsg(json));
    }
}
