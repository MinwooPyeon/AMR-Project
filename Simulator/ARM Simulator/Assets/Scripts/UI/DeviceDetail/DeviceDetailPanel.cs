using UnityEngine;
using UnityEngine.UI;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

public class DeviceDetailPanel : MonoBehaviour
{
    public Text timeStampText;
    public Text positionText;
    public Text amrStateText;
    public Text actionStateText;
    public Text issueText;
    public RawImage image;

    private Texture2D _tex;
    private int _resolution;
    private Vector2Int _cachedRes = Vector2Int.zero;

    private void OnEnable()
    {
        _resolution = Managers.Device.SyncManager.cameraManager.resolution;
        RecreateTexture(_resolution, _resolution);
    }
    public void SetData(SensorFrame frame)
    {
        
        timeStampText.text = frame.timestamp.ToString();
        positionText.text = $"X: {frame.position.x}, Y: {frame.position.y}";
        amrStateText.text = frame.amrState.ToString();
        actionStateText.text = frame.actionState.ToString();
        //TODO Error Case
        //TODO Image
        SetImage(frame);

    }

    private void SetImage(SensorFrame frame)
    {
        _tex.SetPixels32(frame.cameraData);
        _tex.Apply(false, false);
        image.texture = _tex;
    }

    private void RecreateTexture(int w, int h)
    {
        _tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
        _tex.wrapMode = TextureWrapMode.Clamp;
        _tex.filterMode = FilterMode.Bilinear;
        _cachedRes = new Vector2Int(w, h);
        image.texture = _tex;
    }
}
