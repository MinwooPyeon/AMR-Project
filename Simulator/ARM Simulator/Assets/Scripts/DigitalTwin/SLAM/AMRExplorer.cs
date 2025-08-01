using UnityEngine;
using System.Collections;

public class AMRExplorer : MonoBehaviour
{
    public float moveSpeed = 1f;
    public float rotateSpeed = 90f; // degrees/sec

    private enum Mode { Idle, MoveForward, Rotate }
    private Mode currentMode = Mode.Idle;

    private float moveTime = 1.5f;
    private float rotateTime = 1.0f;
    private float timer = 0f;

    
    void Update()
    {
        timer += Time.deltaTime;
        transform.position += transform.forward * moveSpeed * Time.deltaTime;
    }

}
