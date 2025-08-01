using UnityEngine;
using UnityEngine.UI;
using System.IO;

public class PaintCanvasWithUICamera : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName;     // e.g. "map.yaml"
    public string imageFileName;    // e.g. "map.pgm" or "map.png"
    public bool loadAsPNG = false;

    [Header("UI 컴포넌트")]
    public RawImage canvasImage;    // Screen Space – Camera Canvas의 RawImage
    public Camera uiCamera;       // Canvas의 Render Camera

    [Header("브러시 설정")]
    public Color brushColor = Color.black;
    public int brushSize = 8;

    private Texture2D canvasTex;
    private float[,] probGrid;
    private Vector2Int texSize;
    private RectTransform rt;

    private YAMLParser yamlParser = new YAMLParser();
    private ImageParser imageParser = new ImageParser();

    void Start()
    {
        rt = canvasImage.rectTransform;

        // 1) YAML 파싱 (해상도·원점·threshold 등) :contentReference[oaicite:2]{index=2}
        yamlParser.ParseYaml(yamlFileName, out var yamlData);

        // 2) 이미지 로드 (probGrid 생성) :contentReference[oaicite:3]{index=3}
        var img = loadAsPNG
            ? imageParser.LoadPNG(imageFileName)
            : imageParser.LoadPGM(imageFileName);

        probGrid = img.probGrid;
        texSize = new Vector2Int(img.width, img.height);
        rt.anchorMin = new Vector2(0.5f, 0.5f); // 중앙 기준
        rt.anchorMax = new Vector2(0.5f, 0.5f);
        rt.pivot = new Vector2(0.5f, 0.5f); // 중심 정렬
        rt.sizeDelta = new Vector2(img.width, img.height); // 텍스처 픽셀 크기 그대로
        rt.anchoredPosition = Vector2.zero; // Canvas 중앙에 위치

        // 3) 캔버스 텍스처 생성
        canvasTex = new Texture2D(texSize.x, texSize.y, TextureFormat.RGBA32, false);
        canvasTex.filterMode = FilterMode.Point; // ← 필수: Point 필터링
        for (int x = 0; x < texSize.x; x++)
            for (int y = 0; y < texSize.y; y++)
            {
                float v = probGrid[x, y];
                canvasTex.SetPixel(x, y, new Color(v, v, v, 1f));
            }
        canvasTex.Apply();

        canvasImage.texture = canvasTex;
    }

    void Update()
    {
        if (!Input.GetMouseButton(0)) return;

        Vector2 mousePos = Input.mousePosition;
        if (!RectTransformUtility.RectangleContainsScreenPoint(rt, mousePos, uiCamera))
            return;

        RectTransformUtility.ScreenPointToLocalPointInRectangle(
            rt, mousePos, uiCamera, out Vector2 local
        );

        Rect rect = rt.rect;
        float u = (local.x - rect.x) / rect.width;
        float v = (local.y - rect.y) / rect.height;
        int px = Mathf.FloorToInt(u * texSize.x);
        int py = Mathf.FloorToInt(v * texSize.y);

        PaintPixel(px, py);
        canvasTex.Apply();  // 여러 픽셀 찍지 않으니 매 프레임 Apply 비용도 작습니다
    }
    void PaintPixel(int x, int y)
    {
        if (x < 0 || x >= texSize.x || y < 0 || y >= texSize.y) return;
        canvasTex.SetPixel(x, y, brushColor);
    }
    void PaintCircle(int cx, int cy)
    {
        int r = brushSize;
        for (int dx = -r; dx <= r; dx++)
            for (int dy = -r; dy <= r; dy++)
                if (dx * dx + dy * dy <= r * r)
                {
                    int x = cx + dx, y = cy + dy;
                    if (x >= 0 && x < texSize.x && y >= 0 && y < texSize.y)
                        canvasTex.SetPixel(x, y, brushColor);
                }
    }

    public void ClearCanvas()
    {
        Color32[] cols = new Color32[texSize.x * texSize.y];
        for (int i = 0; i < cols.Length; i++) cols[i] = Color.white;
        canvasTex.SetPixels32(cols);
        canvasTex.Apply();
    }

    public void SaveCanvas(string fileName = "canvas.png")
    {
        byte[] png = canvasTex.EncodeToPNG();
        string path = Path.Combine(Application.persistentDataPath, fileName);
        File.WriteAllBytes(path, png);
        Debug.Log($"[PaintCanvasWithUICamera] Saved to {path}");
    }
}
