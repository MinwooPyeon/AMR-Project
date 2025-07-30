using UnityEngine;
using System.Collections.Generic;

[DisallowMultipleComponent]
public class LidarSensor : MonoBehaviour
{
    [Header("Scan Settings")]
    public int horizontalResolution = 360;
    public int verticalResolution = 1;
    public float verticalFOV = 10f;
    public float maxDistance = 50f;
    public LayerMask layerMask = ~0;

    [HideInInspector]
    public Vector3[] pointCloud;

    // ← 미리 계산한 로컬 공간 방향 벡터
    [HideInInspector]
    public List<Vector3> localDirections = new List<Vector3>();

    void OnEnable()
    {
        GenerateLocalDirections();
        pointCloud = new Vector3[localDirections.Count];        
    }

#if UNITY_EDITOR
    // 에디터에서 값 바꿀 때마다 방향 재계산
    void OnValidate() => GenerateLocalDirections();
#endif

    private void GenerateLocalDirections()
    {
        localDirections.Clear();

        for (int v = 0; v < verticalResolution; v++)
        {
            float vAngle = Mathf.Lerp(
                -verticalFOV * 0.5f, verticalFOV * 0.5f,
                verticalResolution == 1 ? 0.5f : (float)v / (verticalResolution - 1)
            );
            var vRot = Quaternion.Euler(vAngle, 0f, 0f);

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = h * (360f / horizontalResolution);
                var hRot = Quaternion.Euler(0f, hAngle, 0f);

                // 로컬 공간의 단위 방향 벡터 저장
                localDirections.Add((vRot * hRot) * Vector3.forward);
            }
        }
    }
}
