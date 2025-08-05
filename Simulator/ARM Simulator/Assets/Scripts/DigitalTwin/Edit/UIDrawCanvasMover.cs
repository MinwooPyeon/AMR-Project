using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(RawImage))]
public class UIDrawCanvasMover : MonoBehaviour
{
    [Header("이동")]
    public float moveSpeed = 300f;
    public float fastSpeed = 1000f;

    [Header("확대/축소")]
    public float zoomStep = 0.1f;
    public float minScale = 0.1f;
    public float maxScale = 10f;

    private RectTransform rt;

    void Start()
    {
        rt = GetComponent<RectTransform>();
    }

    void Update()
    {
        Move();
        Zoom();
    }

    void Move()
    {
        float speed = Input.GetKey(KeyCode.LeftShift) ? fastSpeed : moveSpeed;
        Vector2 dir = Vector2.zero;

        if (Input.GetKey(KeyCode.W)) dir.y += 1;
        if (Input.GetKey(KeyCode.S)) dir.y -= 1;
        if (Input.GetKey(KeyCode.A)) dir.x -= 1;
        if (Input.GetKey(KeyCode.D)) dir.x += 1;

        rt.anchoredPosition += dir * speed * Time.deltaTime;
    }

    void Zoom()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.0001f)
        {
            Vector3 newScale = rt.localScale + Vector3.one * scroll * zoomStep;
            newScale = ClampScale(newScale);
            rt.localScale = newScale;
        }
    }

    Vector3 ClampScale(Vector3 scale)
    {
        float clamped = Mathf.Clamp(scale.x, minScale, maxScale);
        return new Vector3(clamped, clamped, clamped);
    }
}
