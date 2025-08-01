using UnityEngine;
using System.Collections;

public class AMRExplorer : MonoBehaviour
{
    public float moveSpeed = 1f;

    
    void Update()
    {
        transform.position += transform.forward * moveSpeed * Time.deltaTime;
    }

}
