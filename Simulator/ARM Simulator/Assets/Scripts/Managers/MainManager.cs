using UnityEngine;

public class MainManager : MonoBehaviour
{
    #region SingleTon
    public static MainManager _Instance { get; private set; }
    public static PrefabManager PrefabManager { get; private set; } = new PrefabManager();

    private void Awake()
    {
        if (_Instance == null)
        {
            _Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }
    #endregion
}
