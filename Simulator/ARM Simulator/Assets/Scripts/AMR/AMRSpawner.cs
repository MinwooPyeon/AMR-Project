using System.Collections.Generic;
using UnityEngine;

public class AMRSpawner : MonoBehaviour
{
    public GameObject AMR;
    private List<Vector3> Poses;
    public void OnLoad(int num)
    {
        Poses = Managers.Map.Chargers;
        int count = 0;
        foreach(var pos in Poses)
        {
            Vector3 spawnPos = pos + Vector3.up;
            if (count == num) break;
            Instantiate(AMR, spawnPos, Quaternion.identity);
            count++;
        }
    }
}
