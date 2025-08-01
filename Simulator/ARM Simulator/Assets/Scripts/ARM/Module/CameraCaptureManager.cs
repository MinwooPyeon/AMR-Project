using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class CameraCaptureManager : MonoBehaviour
{
    [Header("ī�޶� ���� (ID �� Camera)")]
    public Dictionary<int, Camera> cameras = new Dictionary<int, Camera>();

    [Header("��� �ػ�")]
    public int resolution = 224;

    // ID �� RTHandle
    private Dictionary<int, RTHandle> rtHandles = new Dictionary<int, RTHandle>();

    // WaitForEndOfFrame �ν��Ͻ� Ǯ��
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    void OnEnable()
    {
        // Inspector�� RegistCamera�� �̸� cameras�� ä�����ٰ� ����
        foreach (var kv in cameras)
        {
            int id = kv.Key;
            AllocateRTHandle(id);
        }
    }

    void OnDisable()
    {
        foreach (var handle in rtHandles.Values)
            RTHandles.Release(handle);
        rtHandles.Clear();
    }

    /// <summary>
    /// �ܺο��� ī�޶� ������ �� ȣ��
    /// </summary>
    public void RegistCamera(int id, Camera cam)
    {
        cameras[id] = cam;
        AllocateRTHandle(id);
    }

    /// <summary>
    /// �ܺο��� ī�޶� ���ŵ� �� ȣ��
    /// </summary>
    public void UnregistCamera(int id)
    {
        if (cameras.Remove(id) && rtHandles.TryGetValue(id, out var h))
        {
            RTHandles.Release(h);
            rtHandles.Remove(id);
        }
    }

    private void AllocateRTHandle(int id)
    {
        if (rtHandles.ContainsKey(id)) return;
        var handle = RTHandles.Alloc(
            resolution, resolution,
            colorFormat: GraphicsFormat.R8G8B8A8_UNorm,
            useDynamicScale: true,
            name: $"CamCapture_{id}"
        );
        rtHandles[id] = handle;
    }

    /// <summary>
    /// �ܺο��� ȣ��: �񵿱� ĸó ��û
    /// </summary>
    public void RequestCapture(int id, long timestamp)
    {
        if (!cameras.ContainsKey(id)) return;
        StartCoroutine(CaptureCameraAsync(id, cameras[id], timestamp));
    }

    private IEnumerator CaptureCameraAsync(int id, Camera cam, long timestamp)
    {
        // GPU �������� ���� �� ����
        yield return FrameEnd;

        var rt = rtHandles[id];
        cam.targetTexture = rt;
        cam.Render();
        cam.targetTexture = null;

        // �񵿱� GPU��CPU ����
        AsyncGPUReadback.Request(
            rt, 0, TextureFormat.RGB24,
            req => OnCompleteReadback(req, id, timestamp)
        );
    }

    private void OnCompleteReadback(AsyncGPUReadbackRequest request, int id, long timestamp)
    {
        if (request.hasError)
        {
            Debug.LogWarning($"[CameraCapture] Readback Error (ID={id})");
            return;
        }

        // Color32 �� �׷��̽����� ó�� (Job ���)
        var pixels = request.GetData<Color32>().ToArray();
        float[] gray = ProcessPixelsWithJob(pixels);

        // ���� �ݹ�
        Managers.Data.OnCameraCaptured(id, gray, timestamp);
    }

    private float[] ProcessPixelsWithJob(Color32[] pixels)
    {
        int n = pixels.Length;
        var input = new NativeArray<Color32>(pixels, Allocator.TempJob);
        var output = new NativeArray<float>(n, Allocator.TempJob);

        var job = new PixelProcessingJob
        {
            inputPixels = input,
            output = output
        };
        var handle = job.Schedule(n, 64);
        handle.Complete();

        float[] result = output.ToArray();
        input.Dispose();
        output.Dispose();
        return result;
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
