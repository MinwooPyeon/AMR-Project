using UnityEngine;
using System.Collections.Generic;

public class MapMeshCombiner : MonoBehaviour
{
    [Header("���� (StreamingAssets)")]
    public string yamlFileName;    // ex) "map.yaml"
    public string imageFileName;   // ex) "map.pgm" or "map.png"
    public bool loadAsPNG = false; // true��PNG, false��PGM

    [Header("������")]
    public GameObject obstaclePrefab;

    // �ļ� �ν��Ͻ�
    private YAMLParser yamlParser;
    private ImageParser imageParser;

    // �Ľ̵� ������
    private YamlFile yamlData;
    private ImageFile imageData;

    void Awake()
    {
        yamlParser = new YAMLParser();
        imageParser = new ImageParser();
    }

    void Start()
    {
        LoadYaml();           // ��Ÿ������ �б� :contentReference[oaicite:10]{index=10}
        LoadImage();          // �̹��� & probGrid �б� :contentReference[oaicite:11]{index=11}
        CombineAllObstacles();// CombineInstance �� �޽� ����
    }

    /// <summary>
    /// YAML �� resolution, origin, occThresh, zones �Ľ�
    /// </summary>
    void LoadYaml()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        Debug.Log($"[MapMeshCombiner] YAML loaded: resolution={yamlData.resolution}, origin={yamlData.origin}, occThresh={yamlData.occThresh}");
    }

    /// <summary>
    /// PGM/PNG �о ImageFile Ȯ�� (width, height, probGrid)
    /// </summary>
    void LoadImage()
    {
        imageData = loadAsPNG
            ? imageParser.LoadPNG(imageFileName)  // PNG �ε� :contentReference[oaicite:12]{index=12}
            : imageParser.LoadPGM(imageFileName); // PGM �ε� :contentReference[oaicite:13]{index=13}

        Debug.Log($"[MapMeshCombiner] Image loaded: {imageData.width}��{imageData.height}");
    }

    /// <summary>
    /// probGrid �� occThresh �� �ȼ��� CombineInstance �� ��Ƽ� �ϳ��� Mesh ����
    /// </summary>
    void CombineAllObstacles()
    {
        // ���� �޽�����Ƽ����
        var mf = obstaclePrefab.GetComponent<MeshFilter>();
        var mr = obstaclePrefab.GetComponent<MeshRenderer>();
        Mesh baseMesh = mf.sharedMesh;
        Material mat = mr.sharedMaterial;

        var combines = new List<CombineInstance>();
        int w = imageData.width;
        int h = imageData.height;
        float res = yamlData.resolution; // �� ũ��
        Vector3 org = yamlData.origin;     // ���� ����
        float thresh = yamlData.occThresh;  // �Ӱ谪

        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                if (imageData.probGrid[x, y] > thresh)
                    continue;

                // �ȼ� �� ���� ��ǥ ��ȯ
                Vector3 pos = new Vector3(
                    org.x + x * res,
                    0f,
                    org.z + y * res
                );

                combines.Add(new CombineInstance
                {
                    mesh = baseMesh,
                    transform = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one * res)
                });
            }
        }

        if (combines.Count == 0)
        {
            Debug.LogWarning("[MapMeshCombiner] No occupied cells found.");
            return;
        }

        // �ϳ��� �޽÷� ��ġ��
        Mesh combinedMesh = new Mesh();
        combinedMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        combinedMesh.CombineMeshes(combines.ToArray(), mergeSubMeshes: true, useMatrices: true);

        // �������� GameObject ����
        var go = new GameObject("CombinedObstacles", typeof(MeshFilter), typeof(MeshRenderer));
        go.transform.parent = transform;
        go.GetComponent<MeshFilter>().mesh = combinedMesh;
        go.GetComponent<MeshRenderer>().material = mat;

        Debug.Log($"[MapMeshCombiner] Combined mesh with {combines.Count} instances.");
    }
}
