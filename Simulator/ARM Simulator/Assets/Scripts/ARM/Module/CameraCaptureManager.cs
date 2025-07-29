using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Rendering;

public class CameraCaptureManager : MonoBehaviour
{
    public List<Camera> cameras;
    public int DeviceCount => cameras.Count;
    public int resolution = 224;

    public void CaptureDevice(int index, long timestamp)
    {
        if (index < 0 || index >= cameras.Count) return;
        StartCoroutine(CaptureCameraAsync(cameras[index], timestamp));
    }

    private IEnumerator CaptureCameraAsync(Camera cam, long timestamp)
    {
        yield return new WaitForEndOfFrame();

        RenderTexture rt = new RenderTexture(resolution, resolution, 24);
        cam.targetTexture = rt;
        cam.Render();

        AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24, request => OnCompleteReadback(request, cam.name, timestamp));

        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
    }

    private void OnCompleteReadback(AsyncGPUReadbackRequest request, string camName, long timestamp)
    {
        if (request.hasError)
        {
            Debug.LogWarning($"[Camera:{camName}] GPU Readback Error at {timestamp}");
            return;
        }

        var data = request.GetData<Color32>();
        Color32[] pixels = data.ToArray();

        float[] grayscaleFrame = ProcessPixelsWithJob(pixels);

        Debug.Log($"[Camera:{camName}] Captured {grayscaleFrame.Length} floats at {timestamp}");
        // TODO: Tensor 변환, 전송 등 후처리 (timestamp 포함)
    }

    private float[] ProcessPixelsWithJob(Color32[] pixels)
    {
        var input = new NativeArray<Color32>(pixels, Allocator.TempJob);
        var output = new NativeArray<float>(pixels.Length, Allocator.TempJob);

        var job = new PixelProcessingJob
        {
            inputPixels = input,
            output = output
        };

        var handle = job.Schedule(pixels.Length, 64);
        handle.Complete();

        float[] result = output.ToArray();

        input.Dispose();
        output.Dispose();

        return result;
    }
}