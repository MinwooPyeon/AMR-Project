using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ModuleSyncManager : MonoBehaviour
{
    [Header("매니저 참조")]
    public CameraCaptureManager cameraManager;

    [Header("프레임 예산 (초)")]
    public float frameBudget = 0.005f; // 5ms

    [Header("모듈 최소 처리 주기 (초)")]
    public float updateInterval = 1f;  // 1초

    // 내부 스케줄링 데이터
    private List<string> sortedDeviceIds = new List<string>();
    private Dictionary<string, float> lastProcessTime = new Dictionary<string, float>();
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    public void OnStart()
    {
        StartCoroutine(ProcessDevices());
    }

    private IEnumerator ProcessDevices()
    {
        while (true)
        {
            float now = Time.time;
            float startTime = Time.realtimeSinceStartup;

            sortedDeviceIds = cameraManager.cameras.Keys
            .OrderBy(id => id)
            .ToList();
            // 1) 1초 경과한(또는 아직 한번도 처리되지 않은) 모듈 우선
            var dueModules = sortedDeviceIds
                .Where(id => now - lastProcessTime[id] >= updateInterval)
                .OrderBy(id => lastProcessTime[id]) // 가장 오래된 순
                .ToList();

            // 3) 예산 초과할 때까지 순차 실행
            foreach (var moduleId in dueModules)
            {
                // 요청만 던짐
                long timestamp = System.DateTimeOffset.Now.ToUnixTimeMilliseconds();
                cameraManager.RequestCapture(moduleId, timestamp);
                //lidarManager.RequestScan(moduleId, timestamp);

                // 처리 시간 기록
                lastProcessTime[moduleId] = now;

                if (Time.realtimeSinceStartup - startTime > frameBudget)
                    break;
            }

            // 4) 다음 프레임까지 대기
            yield return FrameEnd;
        }
    }

    // 모듈 추가
    public void RegistModule(string id, GameObject amr)
    {
        //lidarManager.RegisterSensor(id,amr.transform.GetChild(4).GetComponent<LidarSensor>());
        cameraManager.RegistCamera(
            id,
            amr.transform.GetChild(6).GetChild(0).GetComponent<Camera>()
        );

        // 새로 추가된 모듈도 1초 경과 상태로 초기화
        lastProcessTime[id] = Time.time - updateInterval;

        // 업데이트 대상 리스트 갱신
        sortedDeviceIds = cameraManager.cameras.Keys
            .OrderBy(x => x)
            .ToList();
    }

    // 모듈 제거
    public void UnregistModule(string id)
    {
        cameraManager.UnregistCamera(id);
        //lidarManager.UnregisterSensor(id);
        lastProcessTime.Remove(id);

        sortedDeviceIds = cameraManager.cameras.Keys
            .OrderBy(x => x)
            .ToList();
    }
}
