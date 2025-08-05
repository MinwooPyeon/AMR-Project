using UnityEngine;
using UnityEngine.Rendering;

public class JsonBuilder : MonoBehaviour
{
    public string ImageBuild(Texture2D texture)
    {
        byte[] bytes = texture.EncodeToPNG();
        string payload = System.Convert.ToBase64String(bytes);

    }
}
