using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

public class JsonBuilder : MonoBehaviour
{
    public string EditMapMsgBuild(Texture2D texture)
    {
        byte[] bytes = texture.EncodeToPNG();
        string base64 = System.Convert.ToBase64String(bytes);
        var sb = new StringBuilder();
        sb.Append("{");
        sb.Append($"\"img\":\"{base64}\"");
        sb.Append("}");

        string payload = sb.ToString();

        return payload;
    }

    public static string BuildStatusJson(StateData data)
    {
        var sb = new StringBuilder();
        sb.Append("{");

        sb.Append($"\"serial\":\"{data.SerialNumber}\",");
        sb.Append($"\"state\":\"{data.AmrState}\",");
        sb.Append($"\"x\":{data.Position.x},");
        sb.Append($"\"y\":{data.Position.y},");
        sb.Append($"\"speed\":{data.Speed},");
        sb.Append($"\"angle\":{data.Angle}");

        sb.Append("}");
        return sb.ToString();
    }
}
