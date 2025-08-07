using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

[BurstCompile]
struct TransformJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<int> sensorOffsets;
    [ReadOnly] public NativeArray<int> sensorCounts;
    [ReadOnly] public NativeArray<Vector3> localDirections;
    [ReadOnly] public NativeArray<Vector3> sensorPositions;
    [ReadOnly] public NativeArray<Quaternion> sensorRotations;
    [ReadOnly] public NativeArray<float> sensorMaxDistances;
    [ReadOnly] public NativeArray<int> sensorLayerMasks;
    [NativeDisableParallelForRestriction]
    public NativeArray<RaycastCommand> commands;

    public void Execute(int si)
    {
        int offset = sensorOffsets[si];
        int count = sensorCounts[si];
        var origin = sensorPositions[si];
        var rot = sensorRotations[si];
        float md = sensorMaxDistances[si];
        int mask = sensorLayerMasks[si];
        for (int i = 0; i < count; i++)
        {
            Vector3 worldDir = rot * localDirections[offset + i];
            RaycastCommand rc = new RaycastCommand();
            rc.from = origin;
            rc.direction = worldDir;
            rc.distance = md;
            rc.queryParameters.layerMask = mask;
            rc.queryParameters.hitTriggers = QueryTriggerInteraction.Ignore;
            commands[offset + i] = rc;
        }
    }
}

[BurstCompile]
struct DistributeJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<RaycastHit> points;
    [ReadOnly] public NativeArray<RaycastCommand> commands;
    public NativeArray<Vector3> outPoints;

    public void Execute(int i)
    {
        var hit = points[i];
        var cmd = commands[i];

        if (hit.distance > 0)
            outPoints[i] = hit.point;
        else
            outPoints[i] = cmd.from + cmd.direction * cmd.distance;
    }
}
