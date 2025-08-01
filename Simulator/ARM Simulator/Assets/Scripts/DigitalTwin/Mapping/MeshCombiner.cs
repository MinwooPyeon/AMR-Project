using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class MeshCombiner : MonoBehaviour
{
    /// <summary>
    /// mask[x,y] == true �� ������ prefab ��ü(�ڽ� ����)�� ��� ����޽���
    /// material ���� �и� �����մϴ�.
    /// </summary>
    /// <param name="prefab">���� ������</param>
    /// <param name="mask">width��height ũ���� ��ֹ� ����ũ</param>
    /// <param name="origin">�� ���� (world space)</param>
    /// <param name="resolution">�� ũ�� (world units)</param>
    /// <param name="goName">��� GameObject �̸�</param>
    public void CombineMask(
        GameObject prefab,
        bool[,] mask,
        Vector3 origin,
        float resolution,
        string goName)
    {
        if (prefab == null || mask == null) return;

        int width = mask.GetLength(0);
        int height = mask.GetLength(1);

        // prefab ������ ��� MeshFilter & Renderer ����
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        var renderers = prefab.GetComponentsInChildren<MeshRenderer>();

        // material �� CombineInstance ����Ʈ
        var matToCombines = new Dictionary<Material, List<CombineInstance>>();

        // ������, �׸��� �� MeshFilter��subMeshIndex���� CombineInstance ����
        foreach (var mf in filters)
        {
            var mr = mf.GetComponent<MeshRenderer>();
            if (mr == null) continue;

            var mesh = mf.sharedMesh;
            var materials = mr.sharedMaterials;

            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!mask[x, y]) continue;

                    Vector3 cellOrigin = origin + new Vector3(x * resolution, 0, y * resolution);

                    for (int si = 0; si < mesh.subMeshCount; si++)
                    {
                        // �ش� subMeshIndex�� ��Ƽ����
                        var mat = si < materials.Length ? materials[si] : materials[0];

                        if (!matToCombines.TryGetValue(mat, out var list))
                        {
                            list = new List<CombineInstance>();
                            matToCombines[mat] = list;
                        }

                        var ci = new CombineInstance
                        {
                            mesh = mesh,
                            subMeshIndex = si,
                            transform = Matrix4x4.TRS(
    cellOrigin + mf.transform.localPosition * resolution + Vector3.up * 0.01f,
    mf.transform.localRotation,
    mf.transform.localScale * resolution)
                        };
                        list.Add(ci);
                    }
                }
        }

        // material ���� �и��� GameObject ����
        foreach (var kv in matToCombines)
        {
            var mat = kv.Key;
            var combines = kv.Value;
            if (combines.Count == 0) continue;

            var combinedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            // subMesh �и��� �����Ϸ��� mergeSubMeshes=false
            combinedMesh.CombineMeshes(combines.ToArray(), false, true);

            var go = new GameObject(goName + "_" + mat.name);
            go.transform.parent = transform;
            var mfOut = go.AddComponent<MeshFilter>();
            mfOut.mesh = combinedMesh;
            var mrOut = go.AddComponent<MeshRenderer>();
            mrOut.material = mat;
        }
    }

    /// <summary>
    /// �� cells ��ǥ���� prefab ��ü(�ڽ� ����)�� ��� ����޽���
    /// material ���� �и� �����մϴ�.
    /// </summary>
    /// <param name="prefab">���� ������</param>
    /// <param name="cells">Vector2Int[x,y] �迭</param>
    /// <param name="origin">�� ���� (world space)</param>
    /// <param name="resolution">�� ũ�� (world units)</param>
    /// <param name="goName">��� GameObject �̸�</param>
    public void CombineCells(
        GameObject prefab,
        Vector2Int[] cells,
        Vector3 origin,
        float resolution,
        string goName)
    {
        if (prefab == null || cells == null || cells.Length == 0) return;

        // prefab ������ ��� MeshFilter & Renderer ����
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        var renderers = prefab.GetComponentsInChildren<MeshRenderer>();

        // material �� CombineInstance ����Ʈ
        var matToCombines = new Dictionary<Material, List<CombineInstance>>();

        foreach (var mf in filters)
        {
            var mr = mf.GetComponent<MeshRenderer>();
            if (mr == null) continue;

            var mesh = mf.sharedMesh;
            var materials = mr.sharedMaterials;

            foreach (var c in cells)
            {
                Vector3 cellOrigin = origin + new Vector3(c.x * resolution, 0, c.y * resolution);

                for (int si = 0; si < mesh.subMeshCount; si++)
                {
                    var mat = si < materials.Length ? materials[si] : materials[0];

                    if (!matToCombines.TryGetValue(mat, out var list))
                    {
                        list = new List<CombineInstance>();
                        matToCombines[mat] = list;
                    }

                    var ci = new CombineInstance
                    {
                        mesh = mesh,
                        subMeshIndex = si,
                        transform = Matrix4x4.TRS(
                            cellOrigin + mf.transform.localPosition * resolution + Vector3.up * 0.2f,
                            mf.transform.localRotation,
                            mf.transform.localScale * resolution)
                    };
                    list.Add(ci);
                }
            }
        }

        // material ���� �и��� GameObject ����
        foreach (var kv in matToCombines)
        {
            var mat = kv.Key;
            var combines = kv.Value;
            if (combines.Count == 0) continue;

            var combinedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            combinedMesh.CombineMeshes(combines.ToArray(), false, true);

            combinedMesh.RecalculateBounds();

            var go = new GameObject(goName + "_" + mat.name);
            go.transform.parent = transform;
            var mfOut = go.AddComponent<MeshFilter>();
            mfOut.mesh = combinedMesh;
            var mrOut = go.AddComponent<MeshRenderer>();
            mrOut.material = mat;

            Debug.Log($"[{goName}] bounds center={combinedMesh.bounds.center}, size={combinedMesh.bounds.size}");

        }
    }
}
