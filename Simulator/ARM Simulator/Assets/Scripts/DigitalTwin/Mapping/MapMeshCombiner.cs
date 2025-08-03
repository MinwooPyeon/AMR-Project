using UnityEngine;
using System.Collections.Generic;

public class MapMeshCombiner : MonoBehaviour
{
    [Header("���� (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName;
    public bool loadAsPNG = false;

    
    [System.Serializable]
    public struct PrefabInfo
    {
        public GameObject prefab;    // ����� ������
        public float height;         // Y�� ���������� Y ������ ��
    }
    public PrefabInfo obstacleInfo;  // �Ϲ� ��ֹ�
    public PrefabInfo chargerInfo;   // ������
    public PrefabInfo loadInfo;      // �ε� ��
    public PrefabInfo dropInfo;      // ��� ��

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

        // 1) Ȯ�� �׸��� ��� ��ֹ�
        CombinePrefabAtProbGrid(obstacleInfo, yamlData.occThresh);

        // 2) Zone ��ǥ ��� charger/load/drop
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
    /// probGrid �� thresh �� �ȼ����� �ϳ��� Mesh�� Combine
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

                // XZ ��ġ + Y ������
                Vector3 pos = new Vector3(
                    org.x + x * res,
                    info.height,             // ���ϴ� ����
                    org.z + y * res
                );
                // �����Ͽ� Y ���� �ݿ�
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
    /// Zone �� ��Ͽ� ���� Combine (������ �� ���� ���� �� Instantiate ��� Combine)
    /// </summary>
    void CombineZonePrefabs(PrefabInfo info, Vector2Int[] cells)
    {
        // zone�� ���� ������ �����Ƿ� ������ Instantiate �ص� ������
        // ���÷� CombineInstance ���
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
