using UnityEngine;
using System.Collections.Generic;

public class CameraController : MonoBehaviour
{
    bool is_loaded = false;

    [SerializeField] private float moveSpeed = 10f;
    [SerializeField] private float rotateSpeed = 10f;

    // 키 → 이동/회전 방향 매핑
    private Dictionary<KeyCode, Vector3> moveMap;
    private Dictionary<KeyCode, Vector3> rotMap;

    public void OnMapLoaded(ImageFile img)
    {
        // 맵 로드 완료 시 초기 위치 세팅
        transform.position = new Vector3(img.width / 2 * Managers.Map.Resolution, (img.height + img.width) / 3f * Managers.Map.Resolution, img.height / 2 * Managers.Map.Resolution);
        is_loaded = true;
    }

    private void Awake()
    {
        // 이동키 매핑
        moveMap = new Dictionary<KeyCode, Vector3>
        {
            { KeyCode.Q, Vector3.up    },
            { KeyCode.E, Vector3.down  },
            { KeyCode.W, Vector3.forward  },
            { KeyCode.S, Vector3.back },
            { KeyCode.A, Vector3.left },
            { KeyCode.D, Vector3.right  },
        };

        // 회전키 매핑 (Euler 회전값)
        rotMap = new Dictionary<KeyCode, Vector3>
        {
            { KeyCode.U, Vector3.up    },
            { KeyCode.O, Vector3.down  },
            { KeyCode.I, Vector3.forward  },
            { KeyCode.J, Vector3.left },
            { KeyCode.K, Vector3.back },
            { KeyCode.L, Vector3.right  },
        };
    }

    private void Update()
    {
        if (!is_loaded) return;

        float dt = Time.deltaTime;

        // 이동 처리
        foreach (var kv in moveMap)
        {
            if (Input.GetKey(kv.Key))
                transform.Translate(kv.Value * moveSpeed * dt, Space.World);
        }

        // 회전 처리
        foreach (var kv in rotMap)
        {
            if (Input.GetKey(kv.Key))
                transform.Rotate(kv.Value * rotateSpeed * dt);
        }
    }
}
