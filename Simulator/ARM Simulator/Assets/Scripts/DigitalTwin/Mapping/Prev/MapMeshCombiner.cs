using UnityEngine;
using System.Collections.Generic;

public class MapMeshCombiner : MonoBehaviour
{
    [Header("ÆÄÀÏ (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName;
    public bool loadAsPNG = true;

    [System.Serializable]
    public struct PrefabInfo
    {
        public GameObject prefab;
        public float height;
    }
    public PrefabInfo obstacleInfo;
    public PrefabInfo chargerInfo;
    public PrefabInfo loadInfo;
    public PrefabInfo dropInfo;
    public PrefabInfo freeInfo;

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
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        imageData = imageParser.LoadPNG(imageFileName);

        CombineByProbRange(obstacleInfo, (p) => p < 0.1f, "Obstacles");
        CombineByProbRange(freeInfo, (p) => p > 0.9f, "FreeSpaces");
        CombineByProbRange(loadInfo, (p) => p >= 0.55f && p < 0.7f, "LoadZones");
        CombineByProbRange(dropInfo, (p) => p >= 0.3f && p < 0.37f, "DropZones");
        CombineByProbRange(chargerInfo, (p) => p >= 0.48f && p < 0.52f, "ChargerZones");
    }

    void CombineByProbRange(PrefabInfo info, System.Func<float, bool> predicate, string groupName)
    {
        var mf = info.prefab.GetComponent<MeshFilter>();
        var mr = info.prefab.GetComponent<MeshRenderer>();
        Mesh baseMesh = mf.sharedMesh;
        Material mat = mr.sharedMaterial;

        var combines = new List<CombineInstance>();
        float res = yamlData.resolution;
        Vector3 org = yamlData.origin;

        for (int x = 0; x < imageData.width; x++)
            for (int y = 0; y < imageData.height; y++)
            {
                float p = imageData.probGrid[x, y];
                if (!predicate(p)) continue;

                Vector3 pos = new Vector3(org.x + x * res, info.height, org.z + y * res);
                Vector3 scale = new Vector3(res, info.height, res);

                combines.Add(new CombineInstance
                {
                    mesh = baseMesh,
                    transform = Matrix4x4.TRS(pos, Quaternion.identity, scale)
                });
            }

        if (combines.Count == 0) return;

        Mesh combined = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
        combined.CombineMeshes(combines.ToArray(), true, true);

        var group = new GameObject(groupName);
        group.transform.parent = transform;

        var go = new GameObject(info.prefab.name + "_Combined",
            typeof(MeshFilter), typeof(MeshRenderer));
        go.transform.parent = group.transform;
        go.GetComponent<MeshFilter>().mesh = combined;
        go.GetComponent<MeshRenderer>().material = mat;
    }
}
