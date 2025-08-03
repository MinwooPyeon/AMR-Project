using UnityEngine;
using System.Collections.Generic;

public class MapMeshCombiner : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName;
    public bool loadAsPNG = false;

    
    [System.Serializable]
    public struct PrefabInfo
    {
        public GameObject prefab;    // 사용할 프리팹
        public float height;         // Y축 오프셋이자 Y 스케일 값
    }
    public PrefabInfo obstacleInfo;  // 일반 장애물
    public PrefabInfo chargerInfo;   // 충전소
    public PrefabInfo loadInfo;      // 로드 존
    public PrefabInfo dropInfo;      // 드롭 존

    private YAMLParser yamlParser;
    private ImageParser imageParser;
    private YamlFile yamlData;
    private ImageFile imageData;

    void Awake()
    {
        yamlParser = new YAMLParser();
        imageParser = new ImageParser();
    }

    void Start()
    {
        LoadYaml();
        LoadImage();

        // 1) 확률 그리드 기반 장애물
        CombinePrefabAtProbGrid(obstacleInfo, yamlData.occThresh);

        // 2) Zone 좌표 기반 charger/load/drop
        CombineZonePrefabs(chargerInfo, yamlData.chargerCells);
        CombineZonePrefabs(loadInfo, yamlData.loadCells);
        CombineZonePrefabs(dropInfo, yamlData.dropCells);
    }

    void LoadYaml()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
    }

    void LoadImage()
    {
        imageData = imageParser.LoadPNG(imageFileName);
    }

    /// <summary>
    /// probGrid ≥ thresh 인 픽셀에서 하나의 Mesh로 Combine
    /// </summary>
    void CombinePrefabAtProbGrid(PrefabInfo info, float thresh)
    {
        var mf = info.prefab.GetComponent<MeshFilter>();
        var mr = info.prefab.GetComponent<MeshRenderer>();
        Mesh baseMesh = mf.sharedMesh;
        Material mat = mr.sharedMaterial;

        var combines = new List<CombineInstance>();
        int w = imageData.width, h = imageData.height;
        float res = yamlData.resolution;
        Vector3 org = yamlData.origin;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
            {
                if (imageData.probGrid[x, y] < thresh) continue;

                // XZ 위치 + Y 오프셋
                Vector3 pos = new Vector3(
                    org.x + x * res,
                    info.height,             // 원하는 높이
                    org.z + y * res
                );
                // 스케일에 Y 높이 반영
                var scale = new Vector3(res, info.height, res);

                combines.Add(new CombineInstance
                {
                    mesh = baseMesh,
                    transform = Matrix4x4.TRS(pos, Quaternion.identity, scale)
                });
            }

        if (combines.Count == 0) return;

        Mesh combined = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };
        combined.CombineMeshes(combines.ToArray(), true, true);

        var go = new GameObject(info.prefab.name + "_Combined",
            typeof(MeshFilter), typeof(MeshRenderer));
        go.transform.parent = transform;
        go.GetComponent<MeshFilter>().mesh = combined;
        go.GetComponent<MeshRenderer>().material = mat;
    }

    /// <summary>
    /// Zone 셀 목록에 대해 Combine (가능한 셀 수가 적을 땐 Instantiate 대신 Combine)
    /// </summary>
    void CombineZonePrefabs(PrefabInfo info, Vector2Int[] cells)
    {
        // zone은 보통 개수가 적으므로 간단히 Instantiate 해도 되지만
        // 예시로 CombineInstance 사용
        var mf = info.prefab.GetComponent<MeshFilter>();
        var mr = info.prefab.GetComponent<MeshRenderer>();
        Mesh baseMesh = mf.sharedMesh;
        Material mat = mr.sharedMaterial;

        var combines = new List<CombineInstance>();
        float res = yamlData.resolution;
        Vector3 org = yamlData.origin;

        foreach (var cell in cells)
        {
            Vector3 pos = new Vector3(
                org.x + cell.x * res,
                info.height,
                org.z + cell.y * res
            );
            var scale = new Vector3(res, info.height, res);

            combines.Add(new CombineInstance
            {
                mesh = baseMesh,
                transform = Matrix4x4.TRS(pos, Quaternion.identity, scale)
            });
        }

        if (combines.Count == 0) return;

        Mesh combined = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };
        combined.CombineMeshes(combines.ToArray(), true, true);

        var go = new GameObject(info.prefab.name + "_ZoneCombined",
            typeof(MeshFilter), typeof(MeshRenderer));
        go.transform.parent = transform;
        go.GetComponent<MeshFilter>().mesh = combined;
        go.GetComponent<MeshRenderer>().material = mat;
    }
}
