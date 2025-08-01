using UnityEngine;
using System.Collections.Generic;

public class MapMeshCombiner : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName;    // ex) "map.yaml"
    public string imageFileName;   // ex) "map.pgm" or "map.png"
    public bool loadAsPNG = false; // true→PNG, false→PGM

    [Header("프리팹")]
    public GameObject obstaclePrefab;

    // 파서 인스턴스
    private YAMLParser yamlParser;
    private ImageParser imageParser;

    // 파싱된 데이터
    private YamlFile yamlData;
    private ImageFile imageData;

    void Awake()
    {
        yamlParser = new YAMLParser();
        imageParser = new ImageParser();
    }

    void Start()
    {
        LoadYaml();           // 메타데이터 읽기 :contentReference[oaicite:10]{index=10}
        LoadImage();          // 이미지 & probGrid 읽기 :contentReference[oaicite:11]{index=11}
        CombineAllObstacles();// CombineInstance 로 메시 결합
    }

    /// <summary>
    /// YAML → resolution, origin, occThresh, zones 파싱
    /// </summary>
    void LoadYaml()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        Debug.Log($"[MapMeshCombiner] YAML loaded: resolution={yamlData.resolution}, origin={yamlData.origin}, occThresh={yamlData.occThresh}");
    }

    /// <summary>
    /// PGM/PNG 읽어서 ImageFile 확보 (width, height, probGrid)
    /// </summary>
    void LoadImage()
    {
        imageData = loadAsPNG
            ? imageParser.LoadPNG(imageFileName)  // PNG 로드 :contentReference[oaicite:12]{index=12}
            : imageParser.LoadPGM(imageFileName); // PGM 로드 :contentReference[oaicite:13]{index=13}

        Debug.Log($"[MapMeshCombiner] Image loaded: {imageData.width}×{imageData.height}");
    }

    /// <summary>
    /// probGrid ≥ occThresh 인 픽셀을 CombineInstance 로 모아서 하나의 Mesh 생성
    /// </summary>
    void CombineAllObstacles()
    {
        // 원본 메쉬·머티리얼
        var mf = obstaclePrefab.GetComponent<MeshFilter>();
        var mr = obstaclePrefab.GetComponent<MeshRenderer>();
        Mesh baseMesh = mf.sharedMesh;
        Material mat = mr.sharedMaterial;

        var combines = new List<CombineInstance>();
        int w = imageData.width;
        int h = imageData.height;
        float res = yamlData.resolution; // 셀 크기
        Vector3 org = yamlData.origin;     // 월드 원점
        float thresh = yamlData.occThresh;  // 임계값

        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                if (imageData.probGrid[x, y] > thresh)
                    continue;

                // 픽셀 → 월드 좌표 변환
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

        // 하나의 메시로 합치기
        Mesh combinedMesh = new Mesh();
        combinedMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        combinedMesh.CombineMeshes(combines.ToArray(), mergeSubMeshes: true, useMatrices: true);

        // 렌더링용 GameObject 생성
        var go = new GameObject("CombinedObstacles", typeof(MeshFilter), typeof(MeshRenderer));
        go.transform.parent = transform;
        go.GetComponent<MeshFilter>().mesh = combinedMesh;
        go.GetComponent<MeshRenderer>().material = mat;

        Debug.Log($"[MapMeshCombiner] Combined mesh with {combines.Count} instances.");
    }
}
