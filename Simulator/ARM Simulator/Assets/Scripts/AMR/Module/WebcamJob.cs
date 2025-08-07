using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;


[BurstCompile]
public struct PixelProcessingJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Color32> inputPixels;
    [WriteOnly] public NativeArray<float> output;

    public void Execute(int index)
    {
        Color32 c = inputPixels[index];
        output[index] = (c.r + c.g + c.b) / (255f * 3f); // grayscale
    }
}