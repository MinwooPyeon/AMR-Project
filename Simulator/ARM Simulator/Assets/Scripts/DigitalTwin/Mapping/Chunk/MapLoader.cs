using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// SLAM ���� �о�鿩 StreamingAssets�� �ִ� YAML�� Map �̹����� �ε��� ��,
/// MeshCombiner�� ûũ ó���� ���� Mesh ������Ʈ�� �����մϴ�.
/// InstancedRenderer ��� �� ���� �����Ͽ� ������ �ش�ȭ�մϴ�.
/// </summary>
public class MapLoader : MonoBehaviour
{
    [Header("���� (StreamingAssets)")]
    public string yamlFileName;    // ex: "map.yaml"
    public string imageFileName;   // ex: "map.pgm" Ȥ�� "map.png"

    [Header("������")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    [Header("Chunked Combiner (���� ��ġ)")]
    public MeshCombiner combiner;  // MeshCombiner ������Ʈ ����

    private YamlFile yaml;
    private ImageFile image;

    void Start()
    {
        //StartCoroutine(SetupAndCombineCoroutine(chunkSize: 32));
    }

    /// <summary>
    /// 1) ��Ÿ������ �ε�
    /// 2) �̹��� �ε�
    /// 3) ûũ ���� ����
    /// </summary>
    //IEnumerator SetupAndCombineCoroutine(int chunkSize)
    //{
    //    // 1) YAML �Ľ�
    //    string ypath = Path.Combine(Application.streamingAssetsPath, yamlFileName);
    //    new YAMLParser().ParseYaml(ypath, out yaml);

    //    // 2) Map �̹��� �ε�
    //    string ipath = Path.Combine(Application.streamingAssetsPath, imageFileName);
    //    var ip = new ImageParser();
    //    image = ip.LoadPNG(ipath);

    //    // 3) �� ���� ����� (�ð�ȭ)
    //    BuildMapQuad();

    //    // 4) ûũ ����: ��ֹ�
    //    bool[,] mask = new bool[image.width, image.height];
    //    for (int y = 0; y < image.height; y++)
    //        for (int x = 0; x < image.width; x++)
    //            mask[x, y] = image.probGrid[x, y] > yaml.occThresh;
    //    //yield return StartCoroutine(
    //    //    combiner.CombineMaskInChunks(
    //    //        obstaclePrefab,
    //    //        mask,
    //    //        yaml.origin,
    //    //        yaml.resolution,
    //    //        chunkSize,
    //    //        "CombinedObstacles"));

    //    //// 5) zones ûũ ����: ������, �ε�, ���
    //    //yield return StartCoroutine(
    //    //    combiner.CombineCellsInChunks(
    //    //        chargerPrefab,
    //    //        yaml.chargerCells,
    //    //        yaml.origin,
    //    //        yaml.resolution,
    //    //        chunkSize,
    //    //        "CombinedCharger"));
    //    //yield return StartCoroutine(
    //    //    combiner.CombineCellsInChunks(
    //    //        loadPrefab,
    //    //        yaml.loadCells,
    //    //        yaml.origin,
    //    //        yaml.resolution,
    //    //        chunkSize,
    //    //        "CombinedLoad"));
    //    //yield return StartCoroutine(
    //    //    combiner.CombineCellsInChunks(
    //    //        dropPrefab,
    //    //        yaml.dropCells,
    //    //        yaml.origin,
    //    //        yaml.resolution,
    //    //        chunkSize,
    //    //        "CombinedDrop"));
    //}

    /// <summary>
    /// ���� XZ ��鿡 �ð�ȭ�մϴ�.
    /// origin�� ���� �ϴ� �𼭸� ��ġ�Դϴ�.
    /// </summary>
    void BuildMapQuad()
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        float w = image.width * yaml.resolution;
        float h = image.height * yaml.resolution;
        Vector3 center = yaml.origin + new Vector3(w * 0.5f, 0, h * 0.5f);
        quad.transform.position = center;
        quad.transform.localScale = new Vector3(w, h, 1f);
        quad.transform.rotation = Quaternion.Euler(90f, 0, 0);
        var mat = new Material(Shader.Find("Unlit/Texture"));
        mat.mainTexture = image.texture;
        quad.GetComponent<MeshRenderer>().material = mat;
    }

    void BuildObstacle()
    {

    }
}