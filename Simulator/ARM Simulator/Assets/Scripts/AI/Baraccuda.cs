using UnityEngine;
using Unity.Barracuda;

public class BarracudaTest : MonoBehaviour
{
    [Tooltip("Inspector에 추가한 .onnx → NNModel 에셋")]
    public NNModel modelAsset;

    private IWorker worker;

    void Start()
    {
        // 모델 로드 및 워커 생성
        var runtimeModel = ModelLoader.Load(modelAsset);
        worker = WorkerFactory.CreateWorker(WorkerFactory.Type.Auto, runtimeModel);

        // 더미 입력 텐서: 배치=1, 높이=224, 너비=224, 채널=3
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
