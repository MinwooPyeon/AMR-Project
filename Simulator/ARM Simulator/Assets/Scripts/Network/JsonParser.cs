using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;

public class JsonParser
{
    public Dictionary<string, string> Parse(string json)
    {
        Dictionary<string, string> dict = new();

        json = json.Trim().TrimStart('{').TrimEnd('}');
        var pairs = json.Split(new char[] { ',' }, StringSplitOptions.RemoveEmptyEntries);

        foreach (var pair in pairs)
        {
            var kv = pair.Split(new[] { ':' }, 2);
            if (kv.Length != 2) continue;

            var key = kv[0].Trim().Trim('"');
            var value = kv[1].Trim().Trim('"');

            dict[key] = value;
        }

        return dict;
    }
}
