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

    void Start()
    {
        SetRandomBehavior();
    }

    void Update()
    {
        timer += Time.deltaTime;

        switch (currentMode)
        {
            case Mode.MoveForward:
                transform.position += transform.forward * moveSpeed * Time.deltaTime;
                break;
            case Mode.Rotate:
                transform.Rotate(Vector3.up, rotateSpeed * Time.deltaTime);
                break;
        }

        if (timer >= (currentMode == Mode.MoveForward ? moveTime : rotateTime))
            SetRandomBehavior();
    }

    void SetRandomBehavior()
    {
        currentMode = (Random.value > 0.5f) ? Mode.MoveForward : Mode.Rotate;
        timer = 0f;
    }
}
