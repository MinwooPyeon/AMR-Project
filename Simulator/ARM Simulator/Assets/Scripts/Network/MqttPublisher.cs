using UnityEngine;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;

public class MqttPublisher : MonoBehaviour
{
    private MqttClient client;
    private readonly string brokerAddress = "192.168.100.141";
    private readonly int brokerPort = 1883;
    private readonly string[] topic = {"EditMap", "VirtualDeviceStatus" };

    private JsonBuilder jb = new JsonBuilder();
    void Start()
    {
        client = new MqttClient(brokerAddress);
        string clientId = System.Guid.NewGuid().ToString();
        client.Connect(clientId);
    }

    public void PublishEditMap(Texture2D texture)
    {
        string message  = jb.EditMapMsgBuild(texture);
        client.Publish(topic[0], Encoding.UTF8.GetBytes(message), MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE,false);
        Debug.Log($"Published Edit Map Json {message.Length}");
    }

    public void PublishStatus(StateData data)
    {
        string message = jb.BuildStatusJson(data);
        client.Publish(topic[1], Encoding.UTF8.GetBytes(message), MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, false);
        Debug.Log($"Published Status Json {message.Length}");
    }

    void OnDestroy()
    {
        if (client != null && client.IsConnected)
            client.Disconnect();
    }
}
