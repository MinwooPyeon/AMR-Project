using System.IO;
using UnityEngine;

public class MapLoader : MonoBehaviour
{
    [Header("���� (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName; // .pgm or .png

    [Header("������")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    [Header("Combiner")]
    public MeshCombiner combiner; // Scene�� ��ġ�� MeshCombiner ��ü

    private YamlFile yaml;
    private ImageFile image;

    void Start()
    {
        // 1) ��Ÿ������ �ε�
        string yamlPath = Path.Combine(Application.streamingAssetsPath, yamlFileName);
        var yamlParser = new YAMLParser();
        yamlParser.ParseYaml(yamlPath, out yaml);

        // 2) �̹��� �ε�
        string imgPath = Path.Combine(Application.streamingAssetsPath, imageFileName);
        var imgParser = new ImageParser();
        if (Path.GetExtension(imageFileName).ToLower() == ".pgm")
            image = imgParser.LoadPGM(imgPath);
        else
            image = imgParser.LoadPNG(imgPath);


        // 3) �� ���� �ð�ȭ
        BuildMapQuad();

        // 4) �޽� �������� �뷮 ��ġ
        CombineAll();
    }

    private void BuildMapQuad()
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        // ��ġ�������� ������ �״�Ρ�
        quad.transform.position = yaml.origin;
        quad.transform.localScale = new Vector3(
            image.width * yaml.resolution,
            image.height * yaml.resolution,
            1f);

        // �� ���⼭ 90�� ȸ���Ͽ� XZ ������� ����ϴ�.
        quad.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

        var mat = new Material(Shader.Find("Unlit/Texture"));
        mat.mainTexture = image.texture;
        quad.GetComponent<MeshRenderer>().material = mat;

    }

    private void CombineAll()
    {
        // ��ֹ� mask ����
        bool[,] mask = new bool[image.width, image.height];
        for (int y = 0; y < image.height; y++)
            for (int x = 0; x < image.width; x++)
                mask[x, y] = image.probGrid[x, y] > yaml.occThresh;

        // Combiner ȣ��
        combiner.CombineMask(obstaclePrefab, mask, yaml.origin, yaml.resolution, "CombinedObstacles");
        combiner.CombineCells(chargerPrefab, yaml.chargerCells, yaml.origin, yaml.resolution, "CombinedCharger");
        combiner.CombineCells(loadPrefab, yaml.loadCells, yaml.origin, yaml.resolution, "CombinedLoad");
        combiner.CombineCells(dropPrefab, yaml.dropCells, yaml.origin, yaml.resolution, "CombinedDrop");
    }
}
