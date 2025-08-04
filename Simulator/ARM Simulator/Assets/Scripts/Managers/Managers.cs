using Unity.VisualScripting;
using UnityEngine;

public class Managers : MonoBehaviour
{
    #region Attribute
    static Managers _instance;
    static ResourceManager _resource = new ResourceManager();
    static DeviceManager _device = new DeviceManager();
    static DataManager _data = new DataManager();
    #endregion
    #region Methods
    public static Managers Manager { get { return _instance; } }
    public static ResourceManager Resource { get { return _resource; } }
    public static DeviceManager Device { get { return _device; } }
    public static DataManager Data { get { return _data; } }


    #endregion
    #region Unity Methods
    private void Awake()
    {
        if (_instance == null)
        {
            GameObject go = GameObject.Find("@Managers");
            if (go == null)
            {
                go = new GameObject { name = "@Managers" };
                go.AddComponent<Managers>();
            }
            DontDestroyOnLoad(go);
            _instance = go.GetComponent<Managers>();
        }
        _device.SyncManager = this.gameObject.GetComponentInChildren<ModuleSyncManager>();
    }
    #endregion
}
