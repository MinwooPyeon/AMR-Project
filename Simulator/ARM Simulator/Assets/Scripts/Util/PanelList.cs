using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class PanelList : MonoBehaviour
{
    public static PanelList _panels;
    public static PanelList Panels
    {
        get { return _panels; }
    }

    

    private void Awake()
    {
        if(_panels == null)
        {
            GameObject go = GameObject.Find("@UI");
            if (go == null)
            {
                go = new GameObject { name = "@UI" };
                go.AddComponent<PanelList>();
            }
            DontDestroyOnLoad(go);
            _panels = go.GetComponent<PanelList>();
        }
    }

    public GameObject DeviceNumberCanvas;
    public GameObject VirtualMapCanvas;
    public GameObject PathEditCanvas;
    public GameObject TimeLapseCanvas;
    public GameObject DeviceInfoCanvas;
    public GameObject MapEditCanvas;
    public GameObject EditorPathEditCanvas;
    public GameObject DeviceDetailCanvas;
}
