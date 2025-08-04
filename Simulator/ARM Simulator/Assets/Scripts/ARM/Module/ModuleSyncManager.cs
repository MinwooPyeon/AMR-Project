using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ModuleSyncManager : MonoBehaviour
{
    [Header("매니저 참조")]
    public CameraCaptureManager cameraManager;
    public LidarManager lidarManager;

    [Header("프레임 예산 (초)")]
    public float frameBudget = 0.005f; // 5ms

    [Header("모듈 최소 처리 주기 (초)")]
    public float updateInterval = 1f;  // 1초

    // 내부 스케줄링 데이터
    private List<string> sortedDeviceIds = new List<string>();
    private Dictionary<string, float> lastProcessTime = new Dictionary<string, float>();
    private static readonly YieldInstruction FrameEnd = new WaitForEndOfFrame();

    void Start()
    {
        // 공통 ID만 뽑아서 정렬
        sortedDeviceIds = cameraManager.cameras.Keys
            .Intersect(lidarManager.Sensors.Keys)
            .OrderBy(id => id)
            .ToList();

        // 초기화: 모든 모듈을 즉시 처리 가능한 상태로
        float now = Time.time;
        foreach (var id in sortedDeviceIds)
            lastProcessTime[id] = now - updateInterval;

        StartCoroutine(ProcessDevices());
    }

    private IEnumerator ProcessDevices()
    {
        while (true)
        {
            float now = Time.time;
            float startTime = Time.realtimeSinceStartup;

            // 1) 1초 경과한(또는 아직 한번도 처리되지 않은) 모듈 우선
            var dueModules = sortedDeviceIds
                .Where(id => now - lastProcessTime[id] >= updateInterval)
                .OrderBy(id => lastProcessTime[id]) // 가장 오래된 순
                .ToList();

            // 2) 예산이 남으면 라운드로빈으로 나머지 채우기
            if (Time.realtimeSinceStartup - startTime < frameBudget)
            {
                var fillCount = sortedDeviceIds.Count - dueModules.Count;
                if (fillCount > 0)
                {
                    // lastProcessTime 기준으로 가장 최근 처리된 순서 뒤부터
                    var other = sortedDeviceIds
                        .Except(dueModules)
                        .OrderBy(id => lastProcessTime[id])
                        .ToList();
                    dueModules.AddRange(other);
                }
            }

            // 3) 예산 초과할 때까지 순차 실행
            int processed = 0;
            foreach (var moduleId in dueModules)
            {
                // 요청만 던짐
                long timestamp = System.DateTimeOffset.Now.ToUnixTimeMilliseconds();
                cameraManager.RequestCapture(moduleId, timestamp);
                lidarManager.RequestScan(moduleId, timestamp);

                // 처리 시간 기록
                lastProcessTime[moduleId] = now;
                processed++;

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
        lidarManager.RegisterSensor(
            id,
            amr.transform.GetChild(4).GetComponent<LidarSensor>()
        );
        cameraManager.RegistCamera(
            id,
            amr.transform.GetChild(6).GetChild(0).GetComponent<Camera>()
        );

        // 새로 추가된 모듈도 1초 경과 상태로 초기화
        lastProcessTime[id] = Time.time - updateInterval;

        // 업데이트 대상 리스트 갱신
        sortedDeviceIds = cameraManager.cameras.Keys
            .Intersect(lidarManager.Sensors.Keys)
            .OrderBy(x => x)
            .ToList();
    }

    // 모듈 제거
    public void UnregistModule(string id)
    {
        cameraManager.UnregistCamera(id);
        lidarManager.UnregisterSensor(id);
        lastProcessTime.Remove(id);

        sortedDeviceIds = cameraManager.cameras.Keys
            .Intersect(lidarManager.Sensors.Keys)
            .OrderBy(x => x)
            .ToList();
    }
}
