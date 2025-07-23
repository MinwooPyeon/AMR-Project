using UnityEngine;

public class Managers : MonoBehaviour
{
    #region Attribute
    static Managers _instance;
    static UIManager _ui = new UIManager();
    static ResourceManager _resource = new ResourceManager();
    static DeviceManager _device = new DeviceManager();
    static DataManager _data = new DataManager();
    #endregion
    #region Methods
    public static Managers Manager { get { return _instance; } }
    public static UIManager UI { get { return _ui; } }
    public static ResourceManager Resource { get { return _resource; } }
    public static DeviceManager Device { get { return _device; } }
    public static DataManager Data { get { return _data; } }
    #endregion
    #region Unity Methods
    static void Init()
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
    }
    #endregion
}
