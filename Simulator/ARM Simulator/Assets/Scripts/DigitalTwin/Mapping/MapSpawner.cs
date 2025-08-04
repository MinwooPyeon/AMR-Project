using UnityEngine;
using System.Collections.Generic;

public class MapSpawner : MonoBehaviour
{
    [Header("���� ��� (StreamingAssets)")]
    public string yamlFileName;      // ex) "map.yaml"
    public string imageFileName;     // ex) "map.pgm" or "map.png"
    public bool loadAsPNG = false;   // true��PNG, false��PGM

    [Header("������")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    // ���� �ļ� �ν��Ͻ�
    private YAMLParser yamlParser;
    private ImageParser imageParser;

    private YamlFile yamlData;
    private ImageFile imgData;

    void Awake()
    {
        yamlParser = new YAMLParser();
        imageParser = new ImageParser();
    }

    void Start()
    {
        LoadYaml();
        LoadImage();
        SpawnOccupancyObjects();
        SpawnZoneObjects();
    }

    /// <summary>
    /// 1) YAML ���Ͽ��� resolution, origin, thresholds, zone ��ǥ �Ľ�
    /// </summary>
    void LoadYaml()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        Debug.Log($"[MapSpawner] YAML loaded: resolution={yamlData.resolution}, origin={yamlData.origin}");
    }

    /// <summary>
    /// 2) PNG/PGM �̹��� �ε� �� width, height, probGrid ����
    /// </summary>
    void LoadImage()
    {
        imgData = imageParser.LoadPNG(imageFileName);

        Debug.Log($"[MapSpawner] Image loaded: size={imgData.width}��{imgData.height}");
    }

    /// <summary>
    /// 3) Ȯ�� �� occupied_thresh �� �ȼ��� ��ֹ�(ť�� ��) ����
    /// </summary>
    void SpawnOccupancyObjects()
    {
        for (int x = 0; x < imgData.width; x++)
        {
            for (int y = 0; y < imgData.height; y++)
            {
                if (imgData.probGrid[x, y] >= yamlData.occThresh)
                {
                    Vector3 worldPos = PixelToWorld(x, y);
                    Instantiate(obstaclePrefab, worldPos, Quaternion.identity, transform);
                }
            }
        }
    }

    /// <summary>
    /// 4) charger/load/drop �� ��ǥ�� ���� ������ ����
    /// </summary>
    void SpawnZoneObjects()
    {
        SpawnZoneList(yamlData.chargerCells, chargerPrefab);
        SpawnZoneList(yamlData.loadCells, loadPrefab);
        SpawnZoneList(yamlData.dropCells, dropPrefab);
    }

    void SpawnZoneList(Vector2Int[] cells, GameObject prefab)
    {
        foreach (var cell in cells)
        {
            Vector3 worldPos = PixelToWorld(cell.x, cell.y);
            Instantiate(prefab, worldPos, Quaternion.identity, transform);
        }
    }

    /// <summary>
    /// �̹��� �ȼ�(x,y) �� ���� ��ǥ ��ȯ
    /// world.x = origin.x + x*resolution  
    /// world.z = origin.z + y*resolution  
    /// (y���� ���� ���� 0)
    /// </summary>
    Vector3 PixelToWorld(int x, int y)
    {
        float wx = yamlData.origin.x + x * yamlData.resolution;
        float wz = yamlData.origin.z + y * yamlData.resolution;
        return new Vector3(wx, 0f, wz);
    }
}
