using UnityEngine;

public class MapSpawner : MonoBehaviour
{
    [Header("파일 경로 (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName;

    [Header("프리팹")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;
    public GameObject freePrefab;

    private YAMLParser yamlParser;
    private ImageParser imageParser;
    private YamlFile yamlData;
    private ImageFile imgData;

    void Awake()
    {
        yamlParser = new YAMLParser();
        imageParser = new ImageParser();
    }

    public void Load()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        imgData = imageParser.LoadPNG(imageFileName);

        SpawnByProbRanges();
    }

    void SpawnByProbRanges()
    {
        GameObject obstacleGroup = new GameObject("Obstacles");
        GameObject freeGroup = new GameObject("FreeSpaces");
        GameObject loadGroup = new GameObject("LoadZones");
        GameObject dropGroup = new GameObject("DropZones");
        GameObject chargerGroup = new GameObject("ChargerZones");

        obstacleGroup.transform.parent = transform;
        freeGroup.transform.parent = transform;
        loadGroup.transform.parent = transform;
        dropGroup.transform.parent = transform;
        chargerGroup.transform.parent = transform;

        for (int x = 0; x < imgData.width; x++)
        {
            for (int y = 0; y < imgData.height; y++)
            {
                float p = imgData.probGrid[x, y];
                Vector3 pos = PixelToWorld(x, y);

                if (p > 0.9f)
                    Instantiate(freePrefab, pos, Quaternion.identity, freeGroup.transform);
                else if (p < 0.1f)
                    Instantiate(obstaclePrefab, pos, Quaternion.identity, obstacleGroup.transform);
                else if (p >= 0.55f && p < 0.7f)
                    Instantiate(loadPrefab, pos, Quaternion.identity, loadGroup.transform);
                else if (p >= 0.48f && p < 0.52f)
                    Instantiate(chargerPrefab, pos, Quaternion.identity, chargerGroup.transform);
                else if (p >= 0.3f && p < 0.37f)
                    Instantiate(dropPrefab, pos, Quaternion.identity, dropGroup.transform);
            }
        }
    }

    Vector3 PixelToWorld(int x, int y)
    {
        float wx = yamlData.origin.x + x * yamlData.resolution;
        float wz = yamlData.origin.z + y * yamlData.resolution;
        return new Vector3(wx, 0f, wz);
    }
}