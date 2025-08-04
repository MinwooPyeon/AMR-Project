using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;

public class CameraCaptureManager : MonoBehaviour
{
    [Header("ī�޶� ���� (ID �� Camera)")]
    public Dictionary<string, Camera> cameras = new Dictionary<string, Camera>();

    [Header("��� �ػ�")]
    public int resolution = 224;

    // ID �� RenderTexture
    private Dictionary<string, RenderTexture> renderTextures = new Dictionary<string, RenderTexture>();

    // WaitForEndOfFrame �ν��Ͻ� Ǯ��
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    void OnEnable()
    {
        // Inspector�� RegistCamera�� �̸� cameras�� ä�����ٰ� ����
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
    /// �ܺο��� ī�޶� ������ �� ȣ��
    /// </summary>
    public void RegistCamera(string id, Camera cam)
    {
        cameras[id] = cam;
        AllocateRenderTexture(id);
    }

    /// <summary>
    /// �ܺο��� ī�޶� ���ŵ� �� ȣ��
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
    /// �ܺο��� ȣ��: �񵿱� ĸó ��û
    /// </summary>
    public void RequestCapture(string id, long timestamp)
    {
        if (!cameras.ContainsKey(id)) return;
        StartCoroutine(CaptureCameraAsync(id, cameras[id], timestamp));
    }

    private IEnumerator CaptureCameraAsync(string id, Camera cam, long timestamp)
    {
        // GPU �������� ���� �� ����
        yield return FrameEnd;

        var rt = renderTextures[id];
        cam.targetTexture = rt;
        cam.Render();
        cam.targetTexture = null;

        // �񵿱� GPU��CPU ����
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

        // Color32 �� �׷��̽����� ó�� (Job ���)
        var pixelData = request.GetData<Color32>();
        int n = pixelData.Length;

        var input = new NativeArray<Color32>(pixelData.ToArray(), Allocator.TempJob);
        var output = new NativeArray<float>(n, Allocator.TempJob);

        var job = new PixelProcessingJob
        {
            inputPixels = input,
            output = output
        };
        var handle = job.Schedule(n, 64);
        handle.Complete();

        float[] gray = output.ToArray();
        input.Dispose();
        output.Dispose();

        StateData state = Managers.Device.Devices[id].gameObject.GetComponent<StateData>();
        // ���� �ݹ�
        Managers.Data.OnCameraCaptured(id, gray, timestamp, state);
    }

    // �׷��̽����� ��ȯ Job
    struct PixelProcessingJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Color32> inputPixels;
        public NativeArray<float> output;

        public void Execute(int i)
        {
            Color32 c = inputPixels[i];
            // ǥ�� ����ġ: 0.299��R + 0.587��G + 0.114��B
            output[i] = (0.299f * c.r + 0.587f * c.g + 0.114f * c.b) / 255f;
        }
    }
}
