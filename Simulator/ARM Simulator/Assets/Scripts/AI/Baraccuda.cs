using UnityEngine;
using Unity.Barracuda;

public class BarracudaTest : MonoBehaviour
{
    [Tooltip("Inspector�� �߰��� .onnx �� NNModel ����")]
    public NNModel modelAsset;

    private IWorker worker;

    void Start()
    {
        // �� �ε� �� ��Ŀ ����
        var runtimeModel = ModelLoader.Load(modelAsset);
        worker = WorkerFactory.CreateWorker(WorkerFactory.Type.Auto, runtimeModel);

        // ���� �Է� �ټ�: ��ġ=1, ����=224, �ʺ�=224, ä��=3
        using (var input = new Tensor(1, 224, 224, 3))
        {
            worker.Execute(input);
            var output = worker.PeekOutput();
            Debug.Log($"[Barracuda] Output shape: {output.shape}");
            output.Dispose();
        }
    }

    void OnDestroy()
    {
        worker?.Dispose();
    }
}
