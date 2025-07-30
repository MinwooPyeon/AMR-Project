using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class ModuleSyncManager : MonoBehaviour
{
    public CameraCaptureManager cameraManager;
    public LidarManager lidarManager;

    public int devicesPerFrame;  // 프레임당 처리할 디바이스 수
    private int currentIndex = 0;

    // 인덱스 <-> deviceId 변환을 위한 리스트
    private List<int> sortedDeviceIds = new();

    void Update()
    {
        int total = Mathf.Min(cameraManager.DeviceCount, lidarManager.DeviceCount);
        devicesPerFrame = total / 60 + 1;
        if (total == 0 || devicesPerFrame <= 0) return;

        // 디바이스 ID 정렬 (ID 오름차순)
        sortedDeviceIds = cameraManager.cameras.Keys
                            .Intersect(lidarManager.Sensors.Keys)
                            .OrderBy(id => id)
                            .ToList();

        long timestamp = System.DateTimeOffset.Now.ToUnixTimeMilliseconds();

        for (int i = 0; i < devicesPerFrame; i++)
        {
            int index = (currentIndex + i) % total;
            int deviceId = sortedDeviceIds[index];

            cameraManager.CaptureDevice(deviceId, timestamp);
            lidarManager.ScanDevice(deviceId, timestamp);
        }

        currentIndex = (currentIndex + devicesPerFrame) % total;
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
