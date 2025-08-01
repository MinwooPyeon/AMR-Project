using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class MeshCombiner : MonoBehaviour
{
    /// <summary>
    /// mask[x,y] == true 인 셀마다 prefab 전체(자식 포함)의 모든 서브메쉬를
    /// material 별로 분리 결합합니다.
    /// </summary>
    /// <param name="prefab">원본 프리팹</param>
    /// <param name="mask">width×height 크기의 장애물 마스크</param>
    /// <param name="origin">맵 원점 (world space)</param>
    /// <param name="resolution">셀 크기 (world units)</param>
    /// <param name="goName">결과 GameObject 이름</param>
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

        // prefab 내부의 모든 MeshFilter & Renderer 수집
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        var renderers = prefab.GetComponentsInChildren<MeshRenderer>();

        // material 별 CombineInstance 리스트
        var matToCombines = new Dictionary<Material, List<CombineInstance>>();

        // 셀마다, 그리고 각 MeshFilter·subMeshIndex마다 CombineInstance 생성
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
                        // 해당 subMeshIndex의 머티리얼
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

        // material 별로 분리된 GameObject 생성
        foreach (var kv in matToCombines)
        {
            var mat = kv.Key;
            var combines = kv.Value;
            if (combines.Count == 0) continue;

            var combinedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            // subMesh 분리를 유지하려면 mergeSubMeshes=false
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
    /// 각 cells 좌표마다 prefab 전체(자식 포함)의 모든 서브메쉬를
    /// material 별로 분리 결합합니다.
    /// </summary>
    /// <param name="prefab">원본 프리팹</param>
    /// <param name="cells">Vector2Int[x,y] 배열</param>
    /// <param name="origin">맵 원점 (world space)</param>
    /// <param name="resolution">셀 크기 (world units)</param>
    /// <param name="goName">결과 GameObject 이름</param>
    public void CombineCells(
        GameObject prefab,
        Vector2Int[] cells,
        Vector3 origin,
        float resolution,
        string goName)
    {
        if (prefab == null || cells == null || cells.Length == 0) return;

        // prefab 내부의 모든 MeshFilter & Renderer 수집
        var filters = prefab.GetComponentsInChildren<MeshFilter>();
        var renderers = prefab.GetComponentsInChildren<MeshRenderer>();

        // material 별 CombineInstance 리스트
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

        // material 별로 분리된 GameObject 생성
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
