using System.IO;
using UnityEngine;

public class MapLoader : MonoBehaviour
{
    [Header("파일 (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName; // .pgm or .png

    [Header("프리팹")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    [Header("Combiner")]
    public MeshCombiner combiner; // Scene에 배치된 MeshCombiner 객체

    private YamlFile yaml;
    private ImageFile image;

    void Start()
    {
        // 1) 메타데이터 로드
        string yamlPath = Path.Combine(Application.streamingAssetsPath, yamlFileName);
        var yamlParser = new YAMLParser();
        yamlParser.ParseYaml(yamlPath, out yaml);

        // 2) 이미지 로드
        string imgPath = Path.Combine(Application.streamingAssetsPath, imageFileName);
        var imgParser = new ImageParser();
        if (Path.GetExtension(imageFileName).ToLower() == ".pgm")
            image = imgParser.LoadPGM(imgPath);
        else
            image = imgParser.LoadPNG(imgPath);


        // 3) 맵 쿼드 시각화
        BuildMapQuad();

        // 4) 메쉬 결합으로 대량 배치
        CombineAll();
    }

    private void BuildMapQuad()
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        // 위치·스케일 설정은 그대로…
        quad.transform.position = yaml.origin;
        quad.transform.localScale = new Vector3(
            image.width * yaml.resolution,
            image.height * yaml.resolution,
            1f);

        // ← 여기서 90도 회전하여 XZ 평면으로 누웁니다.
        quad.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

        var mat = new Material(Shader.Find("Unlit/Texture"));
        mat.mainTexture = image.texture;
        quad.GetComponent<MeshRenderer>().material = mat;

    }

    private void CombineAll()
    {
        // 장애물 mask 생성
        bool[,] mask = new bool[image.width, image.height];
        for (int y = 0; y < image.height; y++)
            for (int x = 0; x < image.width; x++)
                mask[x, y] = image.probGrid[x, y] > yaml.occThresh;

        // Combiner 호출
        combiner.CombineMask(obstaclePrefab, mask, yaml.origin, yaml.resolution, "CombinedObstacles");
        combiner.CombineCells(chargerPrefab, yaml.chargerCells, yaml.origin, yaml.resolution, "CombinedCharger");
        combiner.CombineCells(loadPrefab, yaml.loadCells, yaml.origin, yaml.resolution, "CombinedLoad");
        combiner.CombineCells(dropPrefab, yaml.dropCells, yaml.origin, yaml.resolution, "CombinedDrop");
    }
}
