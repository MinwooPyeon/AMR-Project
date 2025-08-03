using UnityEngine;
using System.Collections.Generic;

public class MapSpawner : MonoBehaviour
{
    [Header("파일 경로 (StreamingAssets)")]
    public string yamlFileName;      // ex) "map.yaml"
    public string imageFileName;     // ex) "map.pgm" or "map.png"
    public bool loadAsPNG = false;   // true→PNG, false→PGM

    [Header("프리팹")]
    public GameObject obstaclePrefab;
    public GameObject chargerPrefab;
    public GameObject loadPrefab;
    public GameObject dropPrefab;

    // 내부 파서 인스턴스
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
    /// 1) YAML 파일에서 resolution, origin, thresholds, zone 좌표 파싱
    /// </summary>
    void LoadYaml()
    {
        yamlParser.ParseYaml(yamlFileName, out yamlData);
        Debug.Log($"[MapSpawner] YAML loaded: resolution={yamlData.resolution}, origin={yamlData.origin}");
    }

    /// <summary>
    /// 2) PNG/PGM 이미지 로드 → width, height, probGrid 생성
    /// </summary>
    void LoadImage()
    {
        imgData = imageParser.LoadPNG(imageFileName);

        Debug.Log($"[MapSpawner] Image loaded: size={imgData.width}×{imgData.height}");
    }

    /// <summary>
    /// 3) 확률 ≥ occupied_thresh 인 픽셀에 장애물(큐브 등) 생성
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
    /// 4) charger/load/drop 존 좌표에 전용 프리팹 생성
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
    /// 이미지 픽셀(x,y) → 월드 좌표 변환
    /// world.x = origin.x + x*resolution  
    /// world.z = origin.z + y*resolution  
    /// (y축은 지면 높이 0)
    /// </summary>
    Vector3 PixelToWorld(int x, int y)
    {
        float wx = yamlData.origin.x + x * yamlData.resolution;
        float wz = yamlData.origin.z + y * yamlData.resolution;
        return new Vector3(wx, 0f, wz);
    }
}
