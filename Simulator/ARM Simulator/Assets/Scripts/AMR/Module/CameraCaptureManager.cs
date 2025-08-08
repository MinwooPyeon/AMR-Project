using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class CameraCaptureManager : MonoBehaviour
{
    [Header("카메라 매핑 (ID → Camera)")]
    public Dictionary<string, Camera> cameras = new Dictionary<string, Camera>();

    [Header("출력 해상도")]
    public int resolution = 64;

    // ID → RenderTexture
    private Dictionary<string, RenderTexture> renderTextures = new Dictionary<string, RenderTexture>();

    // FrameEnd 풀링
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    void OnEnable()
    {
        foreach (var kv in cameras)
            AllocateRenderTexture(kv.Key);
    }

    void OnDisable()
    {
        foreach (var rt in renderTextures.Values)
            rt.Release();
        renderTextures.Clear();
    }

    /// <summary>
    /// 카메라 등록 시 호출
    /// </summary>
    public void RegistCamera(string id, Camera cam)
    {
        cameras[id] = cam;
        AllocateRenderTexture(id);
    }

    /// <summary>
    /// 카메라 해제 시 호출
    /// </summary>
    public void UnregistCamera(string id)
    {
        cameras.Remove(id);
        if (renderTextures.TryGetValue(id, out var rt))
        {
            rt.Release();
            renderTextures.Remove(id);
        }
    }

    private void AllocateRenderTexture(string id)
    {
        if (renderTextures.ContainsKey(id)) return;
        var rt = new RenderTexture(resolution, resolution, 24, RenderTextureFormat.ARGB32);
        rt.Create();
        renderTextures[id] = rt;
    }

    /// <summary>
    /// 캡처 요청: 비동기 렌더 → CPU 복사
    /// </summary>
    public void RequestCapture(string id, long timestamp)
    {
        if (!cameras.ContainsKey(id)) return;
        StartCoroutine(CaptureCameraAsync(id, cameras[id], timestamp));
    }

    private IEnumerator CaptureCameraAsync(string id, Camera cam, long timestamp)
    {
        yield return FrameEnd;

        var rt = renderTextures[id];
        cam.targetTexture = rt;
        cam.Render();
        cam.targetTexture = null;

        AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24,
            req => OnCompleteReadback(req, id, timestamp)
        );
    }

    private void OnCompleteReadback(AsyncGPUReadbackRequest request, string id, long timestamp)
    {
        if (request.hasError)
        {
            Debug.LogWarning($"[CameraCapture] Readback Error (ID={id})");
            return;
        }

        // Color32 배열로 바로 추출
        Color32[] pixels = request.GetData<Color32>().ToArray();

        StateData state = Managers.Device.VirtualDevices[id]
                             .gameObject.GetComponent<StateData>();

        // 최종 콜백 (Color32[] 버전)
        Managers.Data.OnCameraCaptured(id, pixels, timestamp, state);
    }
}
