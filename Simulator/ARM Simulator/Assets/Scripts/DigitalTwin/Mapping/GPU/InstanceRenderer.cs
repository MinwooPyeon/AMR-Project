using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// GPU �ν��Ͻ� ������� ť�긦 ȿ�������� �������մϴ�.
/// �������� �ø��� �Ÿ� �ø��� �����Ͽ�
/// ȭ�鿡 ���̴� �ν��Ͻ��� �׸��ϴ�.
/// </summary>
public class InstancedRenderer : MonoBehaviour
{
    [Header("�ν��Ͻ��� Mesh & Material")]
    public Mesh mesh;
    public Material material;  // GPU Instancing �ɼ� Ȱ��ȭ �ʿ�
    public float instanceScale = 1f;
    public float maxDrawDistance = 100f;

    // ���� ��Ʈ���� ����Ʈ
    private readonly List<Matrix4x4> _matrices = new List<Matrix4x4>();
    private Plane[] _frustumPlanes;

    /// <summary>
    /// �ν��Ͻ� ��Ʈ������ �߰��մϴ�.
    /// </summary>
    public void AddInstance(Matrix4x4 matrix)
    {
        _matrices.Add(matrix);
    }

    /// <summary>
    /// ������ �������� ȣ��Ǿ� �ν��Ͻ� �������� �����մϴ�.
    /// </summary>
    void LateUpdate()
    {
        if (_matrices.Count == 0)
            return;

        Camera cam = Camera.main;
        if (cam == null || mesh == null || material == null)
            return;

        // 1) �� �������� ��� ���
        _frustumPlanes = GeometryUtility.CalculateFrustumPlanes(cam);

        // 2) ���̴� �ν��Ͻ��� ���͸�
        var visible = new List<Matrix4x4>();
        float radius = mesh.bounds.extents.magnitude * instanceScale;
        for (int i = 0; i < _matrices.Count; i++)
        {
            Vector3 pos = _matrices[i].GetColumn(3);
            // �Ÿ� �ø�
            if (Vector3.Distance(cam.transform.position, pos) > maxDrawDistance)
                continue;
            // �������� �ø�
            if (!GeometryUtility.TestPlanesAABB(
                    _frustumPlanes,
                    new Bounds(pos, Vector3.one * radius * 2)))
                continue;
            visible.Add(_matrices[i]);
        }

        // 3) DrawMeshInstanced ȣ��
        const int batchSize = 1023;
        for (int i = 0; i < visible.Count; i += batchSize)
        {
            int count = Mathf.Min(batchSize, visible.Count - i);
            var slice = visible.GetRange(i, count).ToArray();
            Graphics.DrawMeshInstanced(mesh, 0, material, slice);
        }
    }

    /// <summary>
    /// ���� �����ӿ� �ٽ� �ʱ�ȭ�Ϸ��� ȣ���մϴ�.
    /// </summary>
    public void Clear()
    {
        _matrices.Clear();
    }
}


/// <summary>
/// SLAM �� �����͸� �ε��� �� InstancedRenderer�� �̿���
/// ��ֹ��������ҡ��ε������ ����Ʈ�� �ν��Ͻ����� �׸��ϴ�.
/// </summary>

