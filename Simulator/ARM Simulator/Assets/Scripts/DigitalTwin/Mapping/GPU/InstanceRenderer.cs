using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// GPU 인스턴싱 방식으로 큐브를 효율적으로 렌더링합니다.
/// 프러스텀 컬링과 거리 컬링을 적용하여
/// 화면에 보이는 인스턴스만 그립니다.
/// </summary>
public class InstancedRenderer : MonoBehaviour
{
    [Header("인스턴싱할 Mesh & Material")]
    public Mesh mesh;
    public Material material;  // GPU Instancing 옵션 활성화 필요
    public float instanceScale = 1f;
    public float maxDrawDistance = 100f;

    // 월드 매트릭스 리스트
    private readonly List<Matrix4x4> _matrices = new List<Matrix4x4>();
    private Plane[] _frustumPlanes;

    /// <summary>
    /// 인스턴스 매트릭스를 추가합니다.
    /// </summary>
    public void AddInstance(Matrix4x4 matrix)
    {
        _matrices.Add(matrix);
    }

    /// <summary>
    /// 프레임 마지막에 호출되어 인스턴싱 렌더링을 수행합니다.
    /// </summary>
    void LateUpdate()
    {
        if (_matrices.Count == 0)
            return;

        Camera cam = Camera.main;
        if (cam == null || mesh == null || material == null)
            return;

        // 1) 뷰 프러스텀 평면 계산
        _frustumPlanes = GeometryUtility.CalculateFrustumPlanes(cam);

        // 2) 보이는 인스턴스만 필터링
        var visible = new List<Matrix4x4>();
        float radius = mesh.bounds.extents.magnitude * instanceScale;
        for (int i = 0; i < _matrices.Count; i++)
        {
            Vector3 pos = _matrices[i].GetColumn(3);
            // 거리 컬링
            if (Vector3.Distance(cam.transform.position, pos) > maxDrawDistance)
                continue;
            // 프러스텀 컬링
            if (!GeometryUtility.TestPlanesAABB(
                    _frustumPlanes,
                    new Bounds(pos, Vector3.one * radius * 2)))
                continue;
            visible.Add(_matrices[i]);
        }

        // 3) DrawMeshInstanced 호출
        const int batchSize = 1023;
        for (int i = 0; i < visible.Count; i += batchSize)
        {
            int count = Mathf.Min(batchSize, visible.Count - i);
            var slice = visible.GetRange(i, count).ToArray();
            Graphics.DrawMeshInstanced(mesh, 0, material, slice);
        }
    }

    /// <summary>
    /// 다음 프레임에 다시 초기화하려면 호출합니다.
    /// </summary>
    public void Clear()
    {
        _matrices.Clear();
    }
}


/// <summary>
/// SLAM 맵 데이터를 로드한 뒤 InstancedRenderer를 이용해
/// 장애물·충전소·로딩·드롭 포인트를 인스턴싱으로 그립니다.
/// </summary>

