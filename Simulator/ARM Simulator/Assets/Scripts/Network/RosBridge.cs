using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using StringMsg = RosMessageTypes.Std.StringMsg;

public class RosBridge : MonoBehaviour
{
    #region Attribute
    ROSConnection _ros;
    [SerializeField] string _pubTopic = "unity/server";
    [SerializeField] string _subTopic = "server/unity";
    [SerializeField] float publishRate = 1f;
    float timer = 0f;
    #endregion
    #region Methods
    private void OnReceiveMsg(StringMsg msg)
    {
        Debug.Log($"[Server -> Unity] Received message: {msg.data}");
    }
    #endregion
    #region Unity Methods
    private void Awake()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<StringMsg>(_pubTopic);
        _ros.Subscribe<StringMsg>(_subTopic, OnReceiveMsg);
    }

    void Update()
    {
        // 지정된 속도에 맞춰 주기적으로 메시지 전송
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            timer = 0f;
            // StringMsg 생성: 생성자 파라미터에 data 지정 가능
            var msg = new StringMsg("Hello from Unity!");
            _ros.Publish(_pubTopic, msg);
        }
    }
    #endregion
}
