using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.Globalization;

public class PaintCanvasWithPalette : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName = "map.yaml";
    public string imageFileName = "map.png";
    public string saveFileName = "canvas.png";
    public bool loadAsPNG = true;

    [Header("UI 컴포넌트")]
    public RawImage canvasImage;
    public Camera uiCamera;

    [Header("브러시 설정")]
    public Color brushColor = Color.black;
    public int brushSize = 1;

    private Texture2D canvasTex;
    private float[,] probGrid;
    private Vector2Int texSize;
    private RectTransform rt;

    private YAMLParser yamlParser = new YAMLParser();
    private ImageParser imageParser = new ImageParser();

    public void Load()
    {
        rt = canvasImage.rectTransform;
        yamlParser.ParseYaml(yamlFileName, out var yamlData);
        var img = imageParser.LoadPNG(imageFileName);

        probGrid = img.probGrid;
        texSize = new Vector2Int(img.width, img.height);

        rt.anchorMin = new Vector2(0.5f, 0.5f);
        rt.anchorMax = new Vector2(0.5f, 0.5f);
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = new Vector2(img.width, img.height);
        rt.anchoredPosition = Vector2.zero;

        canvasTex = new Texture2D(texSize.x, texSize.y, TextureFormat.RGBA32, false);
        canvasTex.filterMode = FilterMode.Point;

        for (int x = 0; x < texSize.x; x++)
            for (int y = 0; y < texSize.y; y++)
            {
                float v = probGrid[x, y];
                canvasTex.SetPixel(x, y, ColorMapper.ConvertProbToColor(v));
            }
        canvasTex.Apply();

        canvasImage.texture = canvasTex;
    }
    void Update()
    {
        if (!Input.GetMouseButton(0)) return;
        Vector2Int? pixel = GetHoveredPixel();
        if (pixel.HasValue)
        {
            PaintPixel(pixel.Value.x, pixel.Value.y);
            canvasTex.Apply();
        }
    }

    Vector2Int? GetHoveredPixel()
    {
        Vector2 mousePos = Input.mousePosition;
        if (!RectTransformUtility.RectangleContainsScreenPoint(rt, mousePos, uiCamera)) return null;

        RectTransformUtility.ScreenPointToLocalPointInRectangle(rt, mousePos, uiCamera, out Vector2 local);
        Rect rect = rt.rect;
        float u = (local.x - rect.x) / rect.width;
        float v = (local.y - rect.y) / rect.height;
        int px = Mathf.FloorToInt(u * texSize.x);
        int py = Mathf.FloorToInt(v * texSize.y);

        return new Vector2Int(px, py);
    }
    
    void PaintPixel(int x, int y)
    {
        if (x < 0 || x >= texSize.x || y < 0 || y >= texSize.y) return;
        canvasTex.SetPixel(x, y, brushColor);
        probGrid[x, y] = ColorMapper.ConvertColorToProb(brushColor);
    }


    public void ChangeColor(BrushColor color)
    {
        switch (color)
        {
            case BrushColor.White: brushColor = Color.white; break;
            case BrushColor.Black: brushColor = Color.black; break;
            case BrushColor.Red: brushColor = Color.red; break;
            case BrushColor.Blue: brushColor = Color.blue; break;
            case BrushColor.Green: brushColor = Color.green; break;
            case BrushColor.Yellow: brushColor = Color.yellow; break;
        }
    }
    public void SaveCanvas(string fileName = "canvas.png")
    {
        Texture2D saveTex = new Texture2D(texSize.x, texSize.y, TextureFormat.RGBA32, false);
        for (int x = 0; x < texSize.x; x++)
        {
            for (int y = 0; y < texSize.y; y++)
            {
                float prob = probGrid[x, y];
                Color color = new Color(prob, prob, prob, 1f);
                saveTex.SetPixel(x, y, color);
            }
        }
        saveTex.Apply();

        // 2. PNG로 저장
        byte[] pngBytes = saveTex.EncodeToPNG();
        File.WriteAllBytes(saveFileName, pngBytes);

        Debug.Log($"[PaintCanvasWithPalette] Saved PNG to {saveFileName}");
    }
}
