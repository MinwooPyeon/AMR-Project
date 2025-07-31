using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Collections;

public class ModuleSyncManager : MonoBehaviour
{
    public CameraCaptureManager cameraManager;
    public LidarManager lidarManager;

    private int currentIndex = 0;
    private List<int> sortedDeviceIds = new();
    private float frameBudget = 0.005f; // 5ms 예산

    void Start()
    {
        sortedDeviceIds = cameraManager.cameras.Keys
            .Intersect(lidarManager.Sensors.Keys)
            .OrderBy(id => id)
            .ToList();

        StartCoroutine(ProcessDevices());
    }

    private IEnumerator ProcessDevices()
    {
        while (true)
        {
            int total = sortedDeviceIds.Count;
            if (total == 0)
            {
                yield return null;
                continue;
            }

            long timestamp = System.DateTimeOffset.Now.ToUnixTimeMilliseconds();
            float startTime = Time.realtimeSinceStartup;
            int processed = 0;

            for (int i = 0; i < total; i++)
            {
                int index = (currentIndex + i) % total;
                int deviceId = sortedDeviceIds[index];

                cameraManager.CaptureDevice(deviceId, timestamp);
                lidarManager.ScanDevice(deviceId, timestamp);

                processed++;
                if (Time.realtimeSinceStartup - startTime > frameBudget)
                    break;
            }

            currentIndex = (currentIndex + processed) % total;
            yield return null;
        }
    }
    public void RegistModule(int id, GameObject AMR)
    {
        cameraManager.cameras.Add(id, AMR.transform.GetChild(6).GetChild(0).GetComponent<Camera>());
        lidarManager.RegisterSensor(id, AMR.transform.GetChild(4).GetComponent<LidarSensor>());
    }

    public void UnregistModule(int id)
    {
        cameraManager.cameras.Remove(id);
        lidarManager.UnregisterSensor(id);
    }
}
