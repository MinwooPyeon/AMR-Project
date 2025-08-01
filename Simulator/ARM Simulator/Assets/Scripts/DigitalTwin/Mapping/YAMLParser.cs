using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UIElements;

public class YAMLParser
{
    public YamlFile ParseYaml(string yamlPath)
    {
        YamlFile yaml = new YamlFile();
        List<Vector2Int> chargerCells = new List<Vector2Int>();
        List<Vector2Int> loadCells = new List<Vector2Int>();
        List<Vector2Int> dropCells = new List<Vector2Int>();
        string currentZone = null;

        string fullPath = Path.Combine(Application.streamingAssetsPath, yamlPath);
        foreach(var raw in File.ReadAllLines(fullPath))
        {
            var line = raw.Trim();
            if (IsCommentOrEmpty(line)) continue;
            var keyName = GetKeyName(line);

            if (TryParseZoneDirective(keyName, ref currentZone)) continue;
            if (TryParseZoneEntry(line, currentZone, chargerCells, loadCells, dropCells)) continue;

            TryParseKeyValue(line, yaml);
        }

        yaml.chargerCells = chargerCells.ToArray();
        yaml.loadCells = loadCells.ToArray();
        yaml.dropCells = dropCells.ToArray();

        return yaml;
    }

    private bool IsCommentOrEmpty(string line) => line.Length == 0 || line.StartsWith("#");

    private string GetKeyName(string line)
    {
        int idx = line.IndexOf(':');
        if (idx <= 0) return null;
        return line.Substring(0, idx).Trim();
    }

    private bool TryParseZoneDirective(string keyName, ref string currentZone)
    {
        if (keyName == null) return false;
        switch (keyName)
        {
            case "zones":
                currentZone = null;
                return true;
            case "charger":
            case "load":
            case "drop":
                currentZone = keyName;
                return true;
            default:
                return false;
        }
    }

    private bool TryParseZoneEntry(string line, string currentZone,
                                   List<Vector2Int> chargerList,
                                   List<Vector2Int> loadList,
                                   List<Vector2Int> dropList)
    {
        if (currentZone == null || !line.StartsWith("-"))
            return false;

        // "- [20, 10]" ¡æ "20, 10"
        var coord = line
            .TrimStart('-').Trim()
            .TrimStart('[').TrimEnd(']');
        var parts = coord.Split(',');
        if (parts.Length < 2)
            return false;

        if (int.TryParse(parts[0], out int x) &&
            int.TryParse(parts[1], out int y))
        {
            var cell = new Vector2Int(x, y);
            switch (currentZone)
            {
                case "charger": chargerList.Add(cell); break;
                case "load": loadList.Add(cell); break;
                case "drop": dropList.Add(cell); break;
            }
            return true;
        }
        return false;
    }

    private void TryParseKeyValue(string line, YamlFile yaml)
    {
        var kv = line.Split(new[] { ':' }, 2);
        if (kv.Length < 2) return;
        var key = kv[0].Trim();
        var val = kv[1].Trim();

        switch (key)
        {
            case "resolution":
                if (float.TryParse(val, out var r))
                    yaml.resolution = r;
                break;

            case "occupied_thresh":
                if (float.TryParse(val, out var o))
                    yaml.occThresh = o;
                break;

            case "free_thresh":
                if (float.TryParse(val, out var f))
                    yaml.freeThresh = f;
                break;

            case "origin":
                // origin: [x, y, yaw]
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
