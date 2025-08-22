using System;
using System.Text;
using System.Collections.Concurrent;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class MqttSubscriber : MonoBehaviour
{
    private MqttClient client;
    private ReceiveMessageParser parser = new();

    // 메인 스레드 처리용 큐 (topic, payload)
    private readonly ConcurrentQueue<(string topic, string payload)> _queue = new();

    // 브로커 정보
    [SerializeField] private string brokerAddress = "192.168.100.141";
    [SerializeField] private int brokerPort = 1883;

    // 구독 토픽/QoS
    private readonly string[] topics = { "map", "status" };
    private readonly byte[] qosLevels = {
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE,
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE
    };

    private void Start()
    {
        Application.runInBackground = true; // (옵션) 포커스 없어도 통신 유지

        client = new MqttClient(brokerAddress, brokerPort, false, null, null, MqttSslProtocols.None);
        client.MqttMsgPublishReceived += OnMsgReceived_BackgroundThread;

        string clientId = Guid.NewGuid().ToString();
        client.Connect(clientId);
        client.Subscribe(topics, qosLevels);
    }

    // MQTT 라이브러리의 콜백: **백그라운드 스레드**
    private void OnMsgReceived_BackgroundThread(object sender, MqttMsgPublishEventArgs e)
    {
        string topic = e.Topic;
        string payload = Encoding.UTF8.GetString(e.Message);

        // ❗ 여기서는 Unity API 절대 호출 금지
        // 메인 스레드 큐에 넣기만 한다
        _queue.Enqueue((topic, payload));
    }

    // 메인 스레드에서 호출됨
    private void Update()
    {
        // 누적된 메시지를 메인 스레드에서 안전하게 처리
        while (_queue.TryDequeue(out var item))
        {
            switch (item.topic)
            {
                case "map":
                    HandleMap_MainThread(item.payload);
                    break;
                case "status":
                    HandleStatus_MainThread(item.payload);
                    break;
                default:
                    Debug.LogWarning($"Unknown topic '{item.topic}': {item.payload}");
                    break;
            }
        }
    }

    // ⬇️ 아래 두 함수는 **메인 스레드에서만** 호출됨
    private void HandleMap_MainThread(string json)
    {
        // 파싱은 어디서든 가능하지만, Unity 오브젝트 접근은 여기서만
        MapMsg msg = parser.ParseMapMessage(json);
        Debug.Log($"Map 데이터 수신: {json}");
        // TODO: msg로 맵 로드/표시 로직 (Unity API OK)
    }

    private void HandleStatus_MainThread(string json)
    {
        StatusMsg msg = parser.ParseStatusMessage(json);
        // Debug.Log($"Status 데이터 수신: {json}");

        if (Managers.Map.isLoaded)
        {
            // Unity API를 호출해도 안전 (메인 스레드)
            Managers.Device.RegistRealDevice(msg);
            Managers.Device.DeviceActor(msg.serialNumber, msg);
        }
    }

    private void OnApplicationQuit()
    {
        // 종료 처리
        try
        {
            if (client != null && client.IsConnected)
                client.Disconnect();
        }
        catch { /* 무시 */ }
    }

    private void OnDestroy()
    {
        try
        {
            if (client != null && client.IsConnected)
                client.Disconnect();
        }
        catch { /* 무시 */ }
    }
}
