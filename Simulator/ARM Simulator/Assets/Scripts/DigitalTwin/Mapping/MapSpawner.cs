using UnityEngine;

public class MapSpawner : MonoBehaviour
{
    [Header("파일 경로 (StreamingAssets)")]
    public string yamlFileName;
    public string imageFileName;

    public CameraController MainCamera;

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
        Debug.Log($"IMAGE : {imgData.width}  {imgData.height}  YAML : {yamlData.resolution}");
        Managers.Map.ClearPoses();
        Managers.Map.Grid.SetSize(imgData.width, imgData.height);
        SpawnByProbRanges();
        
        MainCamera.OnMapLoaded(imgData);
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
                Vector2 gridPos = new Vector2(x, y);
                Vector3 worldPos = CoordinateCalc.GridToWorld(gridPos, 0, yamlData.resolution);
                if (p > 0.9f)
                {
                    GameObject go = Instantiate(freePrefab, worldPos, Quaternion.identity, freeGroup.transform);
                    SizeScaler.ChangeScaleWithoutHeight(go, yamlData.resolution);
                    Managers.Map.Grid.SetNode(worldPos, gridPos, NODE_TYPE.FREE);
                    //Debug.Log($"{pos}, {NODE_TYPE.FREE}");
                }
                else if (p < 0.1f)
                {
                    GameObject go = Instantiate(obstaclePrefab, worldPos, Quaternion.identity, obstacleGroup.transform);
                    SizeScaler.ChangeScaleWithoutHeight(go, yamlData.resolution);
                    Managers.Map.Grid.SetNode(worldPos, gridPos, NODE_TYPE.OBSTACLE);
                    //Debug.Log($"{pos}, {NODE_TYPE.OBSTACLE}");
                }
                else if (p >= 0.55f && p < 0.7f)
                {
                    GameObject go = Instantiate(loadPrefab, worldPos, Quaternion.identity, loadGroup.transform);
                    SizeScaler.ChangeScaleWithoutHeight(go, yamlData.resolution);
                    Managers.Map.AddLoaderPos(worldPos);
                    Managers.Map.Grid.SetNode(worldPos, gridPos, NODE_TYPE.LOADER);
                    //Debug.Log($"{pos}, {NODE_TYPE.LOADER}");
                }

                else if (p >= 0.48f && p < 0.52f)
                {
                    GameObject go = Instantiate(chargerPrefab, worldPos, Quaternion.identity, chargerGroup.transform);
                    SizeScaler.ChangeScaleWithoutHeight(go, yamlData.resolution);
                    Managers.Map.AddChargerPos(worldPos);
                    Managers.Map.Grid.SetNode(worldPos, gridPos, NODE_TYPE.CHARGER);
                    //Debug.Log($"{pos}, {NODE_TYPE.CHARGER}");
                }

                else if (p >= 0.3f && p < 0.37f)
                {
                    GameObject go = Instantiate(dropPrefab, worldPos, Quaternion.identity, dropGroup.transform);
                    SizeScaler.ChangeScaleWithoutHeight(go, yamlData.resolution);
                    Managers.Map.AddDroperPos(worldPos);
                    Managers.Map.Grid.SetNode(worldPos, gridPos, NODE_TYPE.DROPER);
                    //Debug.Log($"{pos}, {NODE_TYPE.DROPER}");
                }
                
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