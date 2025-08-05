using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class YAMLParser
{
    /// <summary>
    /// ���/������ ó�� �� Metadata �Ľ� �� Zones �Ľ� �� ��� ��ȯ
    /// </summary>
    public void ParseYaml(string yamlPath, out YamlFile yaml)
    {
        yaml = new YamlFile();

        // 1) ���� �б�
        //string fullPath = GetFullPath(yamlPath);
        //if (!File.Exists(fullPath))
        //{
        //    Debug.LogError($"[YAMLParser] ������ �����ϴ�: {fullPath}");
        //    return;
        //}
        var lines = File.ReadAllLines(yamlPath);

        // 2) ��Ÿ������ (Ű:��) �Ľ�
        ParseMetadata(lines, ref yaml);

    }

    //��������������������������������������������������������������������������������������
    // Metadata: resolution, occupied_thresh, free_thresh, origin
    //��������������������������������������������������������������������������������������
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

    //��������������������������������������������������������������������������������������
    // Utility
    //��������������������������������������������������������������������������������������
    private bool IsCommentOrEmpty(string line)
        => string.IsNullOrEmpty(line) || line.StartsWith("#");

}