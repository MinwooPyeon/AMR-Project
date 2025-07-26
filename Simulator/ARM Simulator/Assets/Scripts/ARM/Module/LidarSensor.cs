using UnityEngine;
using System.Collections.Generic;

[DisallowMultipleComponent]
public class LidarSensor : MonoBehaviour
{
    [Header("Scan Settings")]
    [Tooltip("���� ���� ��ĵ ���� ����")]
    public int horizontalResolution = 360;
    [Tooltip("���� ���� ��ĵ ���� ���� (��: 1�� ���� ���� ��ĵ��)")]
    public int verticalResolution = 1;
    [Tooltip("���� ��ĵ ���� (degrees): -verticalFOV/2 ~ +verticalFOV/2")]
    public float verticalFOV = 10f;
    [Tooltip("�� ���� �ִ� �Ÿ� (m)")]
    public float maxDistance = 50f;
    [Tooltip("���� ���� �浹 ���̾� (��: Obstacle, Vehicle)")]
    public LayerMask layerMask = ~0;

    [Header("Performance")]
    [Tooltip("�� ������ ��ĵ ���� (���� �δ� �� false ����)")]
    public bool scanEveryFrame = true;
    [Tooltip("��ĵ �ֱ� (��), scanEveryFrame=false �� ���� ����")]
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

        // ����/���� ���� ����
        for (int v = 0; v < verticalResolution; v++)
        {
            // ���� ����: -FOV/2 �� +FOV/2
            float vAngle = Mathf.Lerp(-verticalFOV / 2f, verticalFOV / 2f, (verticalResolution == 1 ? 0.5f : (float)v / (verticalResolution - 1)));
            Quaternion vRot = Quaternion.Euler(vAngle, 0, 0);

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = h * (360f / horizontalResolution);
                Quaternion hRot = Quaternion.Euler(0, hAngle, 0);

                // ���� ���������� �߻� ����
                Vector3 dir = transform.rotation * hRot * vRot * Vector3.forward;

                if (Physics.Raycast(transform.position, dir, out RaycastHit hit, maxDistance, layerMask))
                    pointCloud.Add(hit.point);
                else
                    pointCloud.Add(transform.position + dir * maxDistance);

                // (����) ����׿� ���� �ð�ȭ
                // Debug.DrawRay(transform.position, dir * Mathf.Min(hit.distance, maxDistance), Color.green, scanInterval);
            }
        }
    }

    // �� �信�� ����ƮŬ���� �ð�ȭ
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        if (pointCloud != null)
            foreach (var p in pointCloud)
                Gizmos.DrawSphere(p, 0.05f);
    }
}
