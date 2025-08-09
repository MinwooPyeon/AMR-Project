using UnityEngine;

public class SizeScaler
{
    public static void ChangeAMRScale(GameObject go, float resolution)
    {
        Vector3 origin = go.transform.localScale;
        go.transform.localScale = new Vector3(origin.x * resolution, origin.y * resolution, origin.z * resolution);
    }

    public static void ChangeScaleWithoutHeight(GameObject go, float resolution)
    {
        Vector3 origin = go.transform.localScale;
        go.transform.localScale = new Vector3(origin.x * resolution, origin.y, origin.z * resolution);
    }
}
