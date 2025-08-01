using System.Collections.Generic;
using UnityEngine.Rendering;

using UnityEngine;
using System.Collections;

/// <summary>
/// 그리드 셀과 zone 셀을 청크 단위로 결합해
/// 정적 Mesh GameObject를 생성하는 컴포넌트
/// </summary>
public class MeshCombiner : MonoBehaviour
{
    /// <summary>
    /// Grid mask를 chunkSize×chunkSize 단위로 묶어 Mesh를 생성
    /// </summary>
    public IEnumerator CombineMaskInChunks(
        GameObject prefab,
        bool[,] mask,
        Vector3 origin,
        float resolution,
        int chunkSize,
        string goName)
    {
        if (prefab == null || mask == null) yield break;
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        int width = mask.GetLength(0), height = mask.GetLength(1);

        for (int cy = 0; cy < height; cy += chunkSize)
        {
            for (int cx = 0; cx < width; cx += chunkSize)
            {
                var matToCombines = new Dictionary<Material, List<CombineInstance>>();
                int maxY = Mathf.Min(height, cy + chunkSize);
                int maxX = Mathf.Min(width, cx + chunkSize);

                for (int y = cy; y < maxY; y++)
                    for (int x = cx; x < maxX; x++)
                        if (mask[x, y])
                        {
                            Vector3 cellOrigin = origin + new Vector3(x * resolution, 0, y * resolution);
                            foreach (var mf in filters)
                            {
                                var mr = mf.GetComponent<MeshRenderer>();
                                if (mr == null) continue;
                                var mesh = mf.sharedMesh;
                                var materials = mr.sharedMaterials;
                                for (int si = 0; si < mesh.subMeshCount; si++)
                                {
                                    var mat = si < materials.Length ? materials[si] : materials[0];
                                    if (!matToCombines.TryGetValue(mat, out var list))
                                        matToCombines[mat] = list = new List<CombineInstance>();
                                    list.Add(new CombineInstance
                                    {
                                        mesh = mesh,
                                        subMeshIndex = si,
                                        transform = Matrix4x4.TRS(
                                            cellOrigin + mf.transform.localPosition * resolution + Vector3.up * 0.01f,
                                            mf.transform.localRotation,
                                            mf.transform.localScale * resolution)
                                    });
                                }
                            }
                        }

                foreach (var kv in matToCombines)
                {
                    var mat = kv.Key;
                    var combines = kv.Value;
                    if (combines.Count == 0) continue;
                    var combinedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
                    combinedMesh.CombineMeshes(combines.ToArray(), false, true);
                    combinedMesh.RecalculateBounds();
                    var go = new GameObject($"{goName}_{mat.name}_{cx}_{cy}");
                    go.transform.parent = transform;
                    var mfOut = go.AddComponent<MeshFilter>(); mfOut.mesh = combinedMesh;
                    var mrOut = go.AddComponent<MeshRenderer>(); mrOut.material = mat;
                }

                yield return null;
            }
        }
    }

    /// <summary>
    /// Zone cells를 chunkSize 단위로 묶어 Mesh를 생성
    /// </summary>
    public IEnumerator CombineCellsInChunks(
        GameObject prefab,
        Vector2Int[] cells,
        Vector3 origin,
        float resolution,
        int chunkSize,
        string goName)
    {
        if (prefab == null || cells == null || cells.Length == 0) yield break;
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        int total = cells.Length;

        for (int i = 0; i < total; i += chunkSize)
        {
            var matToCombines = new Dictionary<Material, List<CombineInstance>>();
            int end = Mathf.Min(total, i + chunkSize);

            for (int idx = i; idx < end; idx++)
            {
                var c = cells[idx];
                Vector3 cellOrigin = origin + new Vector3(c.x * resolution, 0, c.y * resolution);
                foreach (var mf in filters)
                {
                    var mr = mf.GetComponent<MeshRenderer>();
                    if (mr == null) continue;
                    var mesh = mf.sharedMesh;
                    var materials = mr.sharedMaterials;
                    for (int si = 0; si < mesh.subMeshCount; si++)
                    {
                        var mat = si < materials.Length ? materials[si] : materials[0];
                        if (!matToCombines.TryGetValue(mat, out var list))
                            matToCombines[mat] = list = new List<CombineInstance>();
                        list.Add(new CombineInstance
                        {
                            mesh = mesh,
                            subMeshIndex = si,
                            transform = Matrix4x4.TRS(
                                cellOrigin + mf.transform.localPosition * resolution + Vector3.up * 0.01f,
                                mf.transform.localRotation,
                                mf.transform.localScale * resolution)
                        });
                    }
                }
            }

            foreach (var kv in matToCombines)
            {
                var mat = kv.Key;
                var combines = kv.Value;
                if (combines.Count == 0) continue;
                var combinedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
                combinedMesh.CombineMeshes(combines.ToArray(), false, true);
                combinedMesh.RecalculateBounds();
                var go = new GameObject($"{goName}_{mat.name}_batch_{i}");
                go.transform.parent = transform;
                var mfOut = go.AddComponent<MeshFilter>(); mfOut.mesh = combinedMesh;
                var mrOut = go.AddComponent<MeshRenderer>(); mrOut.material = mat;
            }

            yield return null;
        }
    }
}