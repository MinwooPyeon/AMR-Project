using System;
using System.Text;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class MqttSubscriber : MonoBehaviour
{
    private MqttClient client;
    private ReceiveMessageParser parser;
    // 브로커 정보
    private readonly string brokerAddress = "192.168.100.141";
    private readonly int brokerPort = 1883;

    // 구독할 토픽 목록
    private readonly string[] topics = { "map", "status" };
    // 각 토픽의 QoS 레벨 (토픽 배열 순서와 대응)
    private readonly byte[] qosLevels = {
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE,
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE
    };

    private void Start()
    {
        // 1) 클라이언트 생성
        client = new MqttClient(brokerAddress, brokerPort, false, null, null, MqttSslProtocols.None);
        // 2) 메시지 수신 이벤트 등록
        client.MqttMsgPublishReceived += MsgReceived;

        // 3) 브로커 연결
        string clientId = Guid.NewGuid().ToString();
        client.Connect(clientId);

        // 4) 복수 토픽 구독
        client.Subscribe(topics, qosLevels);
        //Debug.Log($"Subscribed to: {string.Join(", ", topics)}");
    }

    // 5) 메시지 도착 시 호출
    private void MsgReceived(object sender, MqttMsgPublishEventArgs e)
    {
        string topic = e.Topic;
        string payload = Encoding.UTF8.GetString(e.Message);

        switch (topic)
        {
            case "map":
                HandleMap(payload);
                break;
            case "status":
                HandleStatus(payload);
                break;
            default:
                Debug.LogWarning($"Unknown topic '{topic}': {payload}");
                break;
        }
    }

    //TOOD Map Psrsing & Save
    private void HandleMap(string json)
    {
        Debug.Log($"Position 데이터 수신: {json}");
        MapMsg msg = parser.ParseMapMessage(json);
        
        
    }

    private void HandleStatus(string json)
    {
        Debug.Log($"Velocity 데이터 수신: {json}");
        StatusMsg msg = parser.ParseStatusMessage(json);

        if (Managers.Map.isLoaded)
        {
            Managers.Device.RegistRealDevice(msg);
            Managers.Device.DeviceActor(msg.serialNumber, msg);
        }
    }

    private void OnDestroy()
    {
        if (client != null && client.IsConnected)
            client.Disconnect();
    }

}
