using System;
using System.Text;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class AMRSubscriber : MonoBehaviour
{
    private MqttClient client;
    // ���Ŀ ����
    private readonly string brokerAddress = "192.168.100.141";
    private int brokerPort = 1883;

    // ������ ���� ���
    private readonly string[] topics = { "position", "status" };
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
        Debug.Log($"Subscribed to: {string.Join(", ", topics)}");
    }

    // 5) �޽��� ���� �� ȣ��
    private void MsgReceived(object sender, MqttMsgPublishEventArgs e)
    {
        string topic = e.Topic;
        string payload = Encoding.UTF8.GetString(e.Message);

        switch (topic)
        {
            case "position":
                HandlePosition(payload);
                break;
            case "status":
                HandleStatus(payload);
                break;
            default:
                Debug.LogWarning($"Unknown topic '{topic}': {payload}");
                break;
        }
    }

    private void HandlePosition(string json)
    {
        // ��: JSON���� �Ѿ�� x,y,z ��ǥ �Ľ�
        // var pos = JsonUtility.FromJson<PositionData>(json);
        Debug.Log($"Position ������ ����: {json}");
    }

    private void HandleStatus(string json)
    {
        // ��: JSON���� �Ѿ�� vx,vy,vz �Ľ�
        // var vel = JsonUtility.FromJson<VelocityData>(json);
        Debug.Log($"Velocity ������ ����: {json}");
    }

    private void OnDestroy()
    {
        if (client != null && client.IsConnected)
            client.Disconnect();
    }

    // (����) JSON �Ľ̿� Ŭ����
    [Serializable]
    private class PositionData
    {
        public float x;
        public float y;
        public float z;
    }

    [Serializable]
    private class VelocityData
    {
        public float vx;
        public float vy;
        public float vz;
    }
}
