using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// SLAM 맵을 읽어들여 StreamingAssets에 있는 YAML과 Map 이미지를 로드한 뒤,
/// MeshCombiner의 청크 처리로 정적 Mesh 오브젝트를 생성합니다.
/// InstancedRenderer 대신 한 번만 결합하여 성능을 극대화합니다.
/// </summary>
public class MapLoader : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName;    // ex: "map.yaml"
    public string imageFileName;   // ex: "map.pgm" 혹은 "map.png"

    [Header("프리팹")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    [Header("Chunked Combiner (씬에 배치)")]
    public MeshCombiner combiner;  // MeshCombiner 컴포넌트 참조

    private YamlFile yaml;
    private ImageFile image;

    void Start()
    {
        //StartCoroutine(SetupAndCombineCoroutine(chunkSize: 32));
    }

    /// <summary>
    /// 1) 메타데이터 로드
    /// 2) 이미지 로드
    /// 3) 청크 단위 결합
    /// </summary>
    //IEnumerator SetupAndCombineCoroutine(int chunkSize)
    //{
    //    // 1) YAML 파싱
    //    string ypath = Path.Combine(Application.streamingAssetsPath, yamlFileName);
    //    new YAMLParser().ParseYaml(ypath, out yaml);

    //    // 2) Map 이미지 로드
    //    string ipath = Path.Combine(Application.streamingAssetsPath, imageFileName);
    //    var ip = new ImageParser();
    //    image = ip.LoadPNG(ipath);

    //    // 3) 맵 쿼드 만들기 (시각화)
    //    BuildMapQuad();

    //    // 4) 청크 결합: 장애물
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

    //    //// 5) zones 청크 결합: 충전소, 로드, 드롭
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
    /// 맵을 XZ 평면에 시각화합니다.
    /// origin은 좌측 하단 모서리 위치입니다.
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