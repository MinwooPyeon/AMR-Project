using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class YAMLParser
{
    /// <summary>
    /// 상대/절대경로 처리 → Metadata 파싱 → Zones 파싱 → 결과 반환
    /// </summary>
    public void ParseYaml(string yamlPath, out YamlFile yaml)
    {
        yaml = new YamlFile();

        // 1) 파일 읽기
        //string fullPath = GetFullPath(yamlPath);
        //if (!File.Exists(fullPath))
        //{
        //    Debug.LogError($"[YAMLParser] 파일이 없습니다: {fullPath}");
        //    return;
        //}
        var lines = File.ReadAllLines(yamlPath);

        // 2) 메타데이터 (키:값) 파싱
        ParseMetadata(lines, ref yaml);

        // 3) zones 파싱
        ParseZones(lines, out yaml.chargerCells, out yaml.loadCells, out yaml.dropCells);

        Debug.Log(
          $"[YAMLParser] Done → resolution:{yaml.resolution} " +
          $"occThresh:{yaml.occThresh} freeThresh:{yaml.freeThresh} " +
          $"origin:{yaml.origin} charger:{yaml.chargerCells.Length} " +
          $"load:{yaml.loadCells.Length} drop:{yaml.dropCells.Length}"
        );
    }

    //───────────────────────────────────────────
    // Metadata: resolution, occupied_thresh, free_thresh, origin
    //───────────────────────────────────────────
    private void ParseMetadata(string[] lines, ref YamlFile yaml)
    {
        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (IsCommentOrEmpty(line) || line.StartsWith("zones"))
                continue;

            int idx = line.IndexOf(':');
            if (idx < 0) continue;

            var key = line.Substring(0, idx).Trim();
            var val = line.Substring(idx + 1).Trim();

            switch (key)
            {
                case "resolution":
                    if (float.TryParse(val, out var r)) yaml.resolution = r;
                    break;
                case "occupied_thresh":
                    if (float.TryParse(val, out var o)) yaml.occThresh = o;
                    break;
                case "free_thresh":
                    if (float.TryParse(val, out var f)) yaml.freeThresh = f;
                    break;
                case "origin":
                    var nums = val.TrimStart('[').TrimEnd(']').Split(',');
                    if (nums.Length >= 3
                        && float.TryParse(nums[0], out var ox)
                        && float.TryParse(nums[1], out var oy)
                        && float.TryParse(nums[2], out var oz))
                    {
                        yaml.origin = new Vector3(ox, 0, oy);
                    }
                    break;
            }
        }
    }

    //───────────────────────────────────────────
    // Zones: charger, load, drop
    //───────────────────────────────────────────
    private void ParseZones(
        string[] lines,
        out Vector2Int[] charger,
        out Vector2Int[] load,
        out Vector2Int[] drop)
    {
        var chargerList = new List<Vector2Int>();
        var loadList = new List<Vector2Int>();
        var dropList = new List<Vector2Int>();

        string currentZone = null;
        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (IsCommentOrEmpty(line))
                continue;

            // directive
            if (line.StartsWith("zones:"))
            {
                currentZone = null; continue;
            }
            if (line.StartsWith("charger:"))
            {
                currentZone = "charger"; continue;
            }
            if (line.StartsWith("load:"))
            {
                currentZone = "load"; continue;
            }
            if (line.StartsWith("drop:"))
            {
                currentZone = "drop"; continue;
            }

            // entry "- [x, y]"
            if (currentZone != null && line.StartsWith("-"))
            {
                var coord = line
                    .TrimStart('-').Trim()
                    .TrimStart('[').TrimEnd(']');
                var parts = coord.Split(',');
                if (parts.Length == 2
                    && int.TryParse(parts[0], out var x)
                    && int.TryParse(parts[1], out var y))
                {
                    var cell = new Vector2Int(x, y);
                    switch (currentZone)
                    {
                        case "charger": chargerList.Add(cell); break;
                        case "load": loadList.Add(cell); break;
                        case "drop": dropList.Add(cell); break;
                    }
                }
            }
        }

        charger = chargerList.ToArray();
        load = loadList.ToArray();
        drop = dropList.ToArray();
    }

    //───────────────────────────────────────────
    // Utility
    //───────────────────────────────────────────
    private bool IsCommentOrEmpty(string line)
        => string.IsNullOrEmpty(line) || line.StartsWith("#");

    private string GetFullPath(string yamlPath)
        => Path.IsPathRooted(yamlPath)
           ? yamlPath
           : Path.Combine(Application.streamingAssetsPath, yamlPath);
}