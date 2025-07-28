using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.Windows.WebCam;

public class Webcam : MonoBehaviour
{
    public Camera targetCamera;
    public float captureInterval = 0.1f;
    public int width = 640;
    public int height = 480;

    private void Start()
    {
        StartCoroutine(CaptureLoop());
    }

    private IEnumerator CaptureLoop()
    {
        while (true)
        {
            yield return new WaitForSeconds(captureInterval);
            CaptureFrame();
        }
    }

    private void CaptureFrame()
    {
        RenderTexture rt = new RenderTexture(width, height, 24);
        targetCamera.targetTexture = rt;
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);

        targetCamera.Render();
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        tex.Apply();

        targetCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        int parentId = transform.parent != null ? transform.parent.GetInstanceID() : gameObject.GetInstanceID();
        string folderPath = Path.Combine(Application.dataPath, "CapturedFrames", parentId.ToString());
        Directory.CreateDirectory(folderPath);

        string filename = $"frame_{System.DateTime.Now:yyyyMMdd_HHmmss_fff}.png";
        string fullPath = Path.Combine(folderPath, filename);

        File.WriteAllBytes(fullPath, tex.EncodeToPNG());

        Debug.Log($"[Capture] Saved frame: {fullPath}");
    }
}
