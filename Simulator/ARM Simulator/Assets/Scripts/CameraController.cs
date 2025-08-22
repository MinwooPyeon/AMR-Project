using UnityEngine;
using System.Collections.Generic;

public class CameraController : MonoBehaviour
{
    bool is_loaded = false;

    [SerializeField] private float moveSpeed = 10f;
    [SerializeField] private float rotateSpeed = 10f;

    // Ű �� �̵�/ȸ�� ���� ����
    private Dictionary<KeyCode, Vector3> moveMap;
    private Dictionary<KeyCode, Vector3> rotMap;

    public void OnMapLoaded(ImageFile img)
    {
        // �� �ε� �Ϸ� �� �ʱ� ��ġ ����
        transform.position = new Vector3(img.width / 2 * Managers.Map.Resolution, (img.height + img.width) / 3f * Managers.Map.Resolution, img.height / 2 * Managers.Map.Resolution);
        is_loaded = true;
    }

    private void Awake()
    {
        // �̵�Ű ����
        moveMap = new Dictionary<KeyCode, Vector3>
        {
            { KeyCode.Q, Vector3.up    },
            { KeyCode.E, Vector3.down  },
            { KeyCode.W, Vector3.forward  },
            { KeyCode.S, Vector3.back },
            { KeyCode.A, Vector3.left },
            { KeyCode.D, Vector3.right  },
        };

        // ȸ��Ű ���� (Euler ȸ����)
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

        // �̵� ó��
        foreach (var kv in moveMap)
        {
            if (Input.GetKey(kv.Key))
                transform.Translate(kv.Value * moveSpeed * dt, Space.World);
        }

        // ȸ�� ó��
        foreach (var kv in rotMap)
        {
            if (Input.GetKey(kv.Key))
                transform.Rotate(kv.Value * rotateSpeed * dt);
        }
    }
}
