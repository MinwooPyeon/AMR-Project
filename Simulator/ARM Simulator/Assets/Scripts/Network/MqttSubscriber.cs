using System;
using System.Text;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class MqttSubscriber : MonoBehaviour
{
    private MqttClient client;
    private ReceiveMessageParser parser;
    // ���Ŀ ����
    private readonly string brokerAddress = "192.168.100.141";
    private readonly int brokerPort = 1883;

    // ������ ���� ���
    private readonly string[] topics = { "map", "status" };
    // �� ������ QoS ���� (���� �迭 ������ ����)
    private readonly byte[] qosLevels = {
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE,
        MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE
    };

    private void Start()
    {
        // 1) Ŭ���̾�Ʈ ����
        client = new MqttClient(brokerAddress, brokerPort, false, null, null, MqttSslProtocols.None);
        // 2) �޽��� ���� �̺�Ʈ ���
        client.MqttMsgPublishReceived += MsgReceived;

        // 3) ���Ŀ ����
        string clientId = Guid.NewGuid().ToString();
        client.Connect(clientId);

        // 4) ���� ���� ����
        client.Subscribe(topics, qosLevels);
        //Debug.Log($"Subscribed to: {string.Join(", ", topics)}");
    }

    // 5) �޽��� ���� �� ȣ��
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
        Debug.Log($"Position ������ ����: {json}");
        MapMsg msg = parser.ParseMapMessage(json);
        
        
    }

    private void HandleStatus(string json)
    {
        Debug.Log($"Velocity ������ ����: {json}");
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
