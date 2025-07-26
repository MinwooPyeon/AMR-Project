using UnityEngine;
using System.Collections.Generic;

[DisallowMultipleComponent]
public class LidarSensor : MonoBehaviour
{
    [Header("Scan Settings")]
    [Tooltip("수평 방향 스캔 레이 개수")]
    public int horizontalResolution = 360;
    [Tooltip("수직 방향 스캔 레이 개수 (예: 1만 쓰면 수평 스캔만)")]
    public int verticalResolution = 1;
    [Tooltip("수직 스캔 범위 (degrees): -verticalFOV/2 ~ +verticalFOV/2")]
    public float verticalFOV = 10f;
    [Tooltip("각 레이 최대 거리 (m)")]
    public float maxDistance = 50f;
    [Tooltip("레이 사이 충돌 레이어 (예: Obstacle, Vehicle)")]
    public LayerMask layerMask = ~0;

    [Header("Performance")]
    [Tooltip("매 프레임 스캔 실행 (성능 부담 시 false 권장)")]
    public bool scanEveryFrame = true;
    [Tooltip("스캔 주기 (초), scanEveryFrame=false 일 때만 적용")]
    public float scanInterval = 0.1f;

    [HideInInspector]
    public List<Vector3> pointCloud = new List<Vector3>();

    float _nextScanTime = 0f;

    void Update()
    {
        if (scanEveryFrame || Time.time >= _nextScanTime)
        {
            if (!scanEveryFrame)
                _nextScanTime = Time.time + scanInterval;
            PerformScan();
        }
    }

    void PerformScan()
    {
        pointCloud.Clear();

        // 수직/수평 이중 루프
        for (int v = 0; v < verticalResolution; v++)
        {
            // 수직 각도: -FOV/2 … +FOV/2
            float vAngle = Mathf.Lerp(-verticalFOV / 2f, verticalFOV / 2f, (verticalResolution == 1 ? 0.5f : (float)v / (verticalResolution - 1)));
            Quaternion vRot = Quaternion.Euler(vAngle, 0, 0);

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = h * (360f / horizontalResolution);
                Quaternion hRot = Quaternion.Euler(0, hAngle, 0);

                // 월드 공간에서의 발사 방향
                Vector3 dir = transform.rotation * hRot * vRot * Vector3.forward;

                if (Physics.Raycast(transform.position, dir, out RaycastHit hit, maxDistance, layerMask))
                    pointCloud.Add(hit.point);
                else
                    pointCloud.Add(transform.position + dir * maxDistance);

                // (선택) 디버그용 레이 시각화
                // Debug.DrawRay(transform.position, dir * Mathf.Min(hit.distance, maxDistance), Color.green, scanInterval);
            }
        }
    }

    // 씬 뷰에서 포인트클라우드 시각화
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        if (pointCloud != null)
            foreach (var p in pointCloud)
                Gizmos.DrawSphere(p, 0.05f);
    }
}
