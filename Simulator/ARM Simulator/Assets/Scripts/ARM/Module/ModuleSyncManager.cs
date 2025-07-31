using UnityEngine;

public class ModuleSyncManager : MonoBehaviour
{
    public CameraCaptureManager cameraManager;
    public LidarManager lidarManager;

    public int devicesPerFrame = 2;  // �����Ӵ� ó���� ����̽� ��

    private int currentIndex = 0;

    void Update()
    {
        int total = Mathf.Min(cameraManager.DeviceCount, lidarManager.DeviceCount);
        if (total == 0 || devicesPerFrame <= 0) return;

        long timestamp = System.DateTimeOffset.Now.ToUnixTimeMilliseconds();

        for (int i = 0; i < devicesPerFrame; i++)
        {
            int index = (currentIndex + i) % total;

            cameraManager.CaptureDevice(index, timestamp);
            lidarManager.ScanDevice(index, timestamp);
        }

        currentIndex = (currentIndex + devicesPerFrame) % total;
    }

    public void RegistModule(GameObject AMR)
    {
        cameraManager.cameras.Add(AMR.transform.GetChild(6).GetChild(0).GetComponent<Camera>());
        lidarManager.RegisterSensor(AMR.transform.GetChild(4).GetComponent<LidarSensor>());
    }
}
