using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;

public class M2MqttTest : MonoBehaviour
{
    private MqttClient client;
    private readonly string brokerAddress = "test.mosquitto.org";
    private readonly string topic = "unity/test";

    void Start()
    {
        client = new MqttClient(brokerAddress);
        client.MqttMsgPublishReceived += OnMessageReceived;
        string clientId = System.Guid.NewGuid().ToString();
        client.Connect(clientId);

        // 备刀
        client.Subscribe(
            new string[] { topic },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE }
        );

        // 林扁利 惯青
        InvokeRepeating(nameof(PublishTestMessage), 1f, 5f);
    }

    void PublishTestMessage()
    {
        var message = $"Hello M2Mqtt @ {System.DateTime.Now:HH:mm:ss}";
        client.Publish(topic, Encoding.UTF8.GetBytes(message));
        Debug.Log($"[M2Mqtt] Published: {message}");
    }

    void OnMessageReceived(object sender, MqttMsgPublishEventArgs e)
    {
        var msg = Encoding.UTF8.GetString(e.Message);
        Debug.Log($"[M2Mqtt] Received: {msg}");
    }

    void OnDestroy()
    {
        if (client != null && client.IsConnected)
            client.Disconnect();
    }
}
