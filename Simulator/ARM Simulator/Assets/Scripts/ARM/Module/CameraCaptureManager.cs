using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;

public class CameraCaptureManager : MonoBehaviour
{
    [Header("카메라 매핑 (ID → Camera)")]
    public Dictionary<int, Camera> cameras = new Dictionary<int, Camera>();

    [Header("출력 해상도")]
    public int resolution = 224;

    // ID → RenderTexture
    private Dictionary<int, RenderTexture> renderTextures = new Dictionary<int, RenderTexture>();

    // WaitForEndOfFrame 인스턴스 풀링
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    void OnEnable()
    {
        // Inspector나 RegistCamera로 미리 cameras가 채워졌다고 가정
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
    /// 외부에서 카메라가 생성될 때 호출
    /// </summary>
    public void RegistCamera(int id, Camera cam)
    {
        cameras[id] = cam;
        AllocateRenderTexture(id);
    }

    /// <summary>
    /// 외부에서 카메라가 제거될 때 호출
    /// </summary>
    public void UnregistCamera(int id)
    {
        cameras.Remove(id);
        if (renderTextures.TryGetValue(id, out var rt))
        {
            rt.Release();
            renderTextures.Remove(id);
        }
    }

    private void AllocateRenderTexture(int id)
    {
        if (renderTextures.ContainsKey(id)) return;

        var rt = new RenderTexture(resolution, resolution, 24, RenderTextureFormat.ARGB32);
        rt.Create();
        renderTextures[id] = rt;
    }

    /// <summary>
    /// 외부에서 호출: 비동기 캡처 요청
    /// </summary>
    public void RequestCapture(int id, long timestamp)
    {
        if (!cameras.ContainsKey(id)) return;
        StartCoroutine(CaptureCameraAsync(id, cameras[id], timestamp));
    }

    private IEnumerator CaptureCameraAsync(int id, Camera cam, long timestamp)
    {
        // GPU 렌더링이 끝난 뒤 실행
        yield return FrameEnd;

        var rt = renderTextures[id];
        cam.targetTexture = rt;
        cam.Render();
        cam.targetTexture = null;

        // 비동기 GPU→CPU 복사
        AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24,
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

        // Color32 → 그레이스케일 처리 (Job 사용)
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

        // 최종 콜백
        Managers.Data.OnCameraCaptured(id, gray, timestamp);
    }

    // 그레이스케일 변환 Job
    struct PixelProcessingJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Color32> inputPixels;
        public NativeArray<float> output;

        public void Execute(int i)
        {
            Color32 c = inputPixels[i];
            // 표준 가중치: 0.299·R + 0.587·G + 0.114·B
            output[i] = (0.299f * c.r + 0.587f * c.g + 0.114f * c.b) / 255f;
        }
    }
}
