using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System.Collections.Generic;
using System.Linq;

public class LidarManager : MonoBehaviour
{
    private Dictionary<int, LidarSensor> _sensorMap = new();
    private Dictionary<int, Vector3[]> _latestPointClouds = new();
    private List<int> _sortedDeviceIds = new();


    public IReadOnlyDictionary<int, LidarSensor> Sensors => _sensorMap;
    public int DeviceCount => _sensorMap.Count;

    private NativeArray<int> _sensorOffsets;
    private NativeArray<int> _sensorCounts;
    private NativeArray<Vector3> _localDirections;
    private NativeArray<Vector3> _sensorPositions;
    private NativeArray<Quaternion> _sensorRotations;
    private NativeArray<float> _sensorMaxDistances;
    private NativeArray<int> _sensorLayerMasks;
    private NativeArray<RaycastCommand> _commands;
    private NativeArray<RaycastHit> _points;
    private NativeArray<Vector3> _outPoints;

    private int _totalRays;

    private void Awake()
    {
       Initialize();
    }

    private void Initialize()
    {
        if (!_commands.IsCreated && _sensorMap.Count > 0)
            RebuildArrays();
    }

    public void RegisterSensor(int deviceId, LidarSensor sensor)
    {
        if (!_sensorMap.ContainsKey(deviceId))
        {
            _sensorMap[deviceId] = sensor;
            RebuildArrays();
        }
    }

    public void UnregisterSensor(int deviceId)
    {
        if (_sensorMap.ContainsKey(deviceId))
        {
            _sensorMap.Remove(deviceId);
            RebuildArrays();
        }
    }

    private void OnDestroy() => DisposeAll();

    private void DisposeAll()
    {
        _commands.DisposeIfCreated();
        _points.DisposeIfCreated();
        _outPoints.DisposeIfCreated();
        _sensorOffsets.DisposeIfCreated();
        _sensorCounts.DisposeIfCreated();
        _localDirections.DisposeIfCreated();
        _sensorPositions.DisposeIfCreated();
        _sensorRotations.DisposeIfCreated();
        _sensorMaxDistances.DisposeIfCreated();
        _sensorLayerMasks.DisposeIfCreated();
    }

    private void RebuildArrays()
    {
        DisposeAll();

        _sortedDeviceIds = _sensorMap.Keys.OrderBy(id => id).ToList();
        int count = _sortedDeviceIds.Count;

        _sensorOffsets = new NativeArray<int>(count, Allocator.Persistent);
        _sensorCounts = new NativeArray<int>(count, Allocator.Persistent);
        _sensorPositions = new NativeArray<Vector3>(count, Allocator.Persistent);
        _sensorRotations = new NativeArray<Quaternion>(count, Allocator.Persistent);
        _sensorMaxDistances = new NativeArray<float>(count, Allocator.Persistent);
        _sensorLayerMasks = new NativeArray<int>(count, Allocator.Persistent);

        _totalRays = 0;
        for (int i = 0; i < count; i++)
        {
            var sensor = _sensorMap[_sortedDeviceIds[i]];
            int rayCount = sensor.localDirections.Count;

            _sensorOffsets[i] = _totalRays;
            _sensorCounts[i] = rayCount;
            _sensorMaxDistances[i] = sensor.maxDistance;
            _sensorLayerMasks[i] = sensor.layerMask;

            _totalRays += rayCount;
        }

        _localDirections = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
        int idx = 0;
        foreach (int id in _sortedDeviceIds)
        {
            foreach (var dir in _sensorMap[id].localDirections)
                _localDirections[idx++] = dir;
        }

        _commands = new NativeArray<RaycastCommand>(_totalRays, Allocator.Persistent);
        _points = new NativeArray<RaycastHit>(_totalRays, Allocator.Persistent);
        _outPoints = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
    }

    public void ScanDevice(int deviceId, long timestamp)
    {
        int index = _sortedDeviceIds.IndexOf(deviceId);
        if (index < 0 || !_sensorMap.ContainsKey(deviceId)) return;

        var sensor = _sensorMap[deviceId];
        int offset = _sensorOffsets[index];
        int count = _sensorCounts[index];

        _sensorPositions[index] = sensor.transform.parent.transform.position;
        _sensorRotations[index] = sensor.transform.parent.transform.rotation;

        var tHandle = ScheduleTransformJob();
        var rHandle = ScheduleRaycastJob(offset, count, tHandle);
        var dHandle = ScheduleDistributeJob(offset, count, rHandle);

        dHandle.Complete();
        WriteBackSensorResults(deviceId, sensor, offset, count, timestamp);
    }

    private JobHandle ScheduleTransformJob()
    {
        var tJob = new TransformJob
        {
            sensorOffsets = _sensorOffsets,
            sensorCounts = _sensorCounts,
            localDirections = _localDirections,
            sensorPositions = _sensorPositions,
            sensorRotations = _sensorRotations,
            sensorMaxDistances = _sensorMaxDistances,
            sensorLayerMasks = _sensorLayerMasks,
            commands = _commands
        };
        return tJob.Schedule(1, 1);
    }

    private JobHandle ScheduleRaycastJob(int offset, int count, JobHandle dependency)
    {
        int batchSize = Mathf.Max(1, count / (SystemInfo.processorCount * 4));
        return RaycastCommand.ScheduleBatch(
            _commands.GetSubArray(offset, count),
            _points.GetSubArray(offset, count),
            batchSize,
            dependency);
    }

    private JobHandle ScheduleDistributeJob(int offset, int count, JobHandle dependency)
    {
        var dJob = new DistributeJob
        {
            points = _points,
            commands = _commands,
            outPoints = _outPoints
        };
        return dJob.Schedule(count, 64, dependency);
    }

    private void WriteBackSensorResults(int deviceId, LidarSensor sensor, int offset, int count, long timestamp)
    {
        Vector3[] points = new Vector3[count];
        for (int i = 0; i < count; i++)
            points[i] = _outPoints[offset + i];

        sensor.pointCloud = points;
        _latestPointClouds[deviceId] = points;

        // ✅ 추가: DataManager에 전달
        var amr = sensor.transform.parent.gameObject;
        var state = amr.GetComponent<StateData>();
        if (state != null)
        {
            Managers.Data.OnLidarScanned(deviceId, points, timestamp, state);
            OccupancyGridManager.Instance.AddLidarPoints(points, sensor);
        }
        else
        {
            Debug.LogWarning($"[LidarManager] StateData not found on AMR {amr.name}");
        }

        //Debug.Log($"[Lidar:{sensor.name}] Scanned {count} points at {timestamp}");
    }


    public Vector3[] GetLastPointCloud(int deviceId)
    {
        return _latestPointClouds.TryGetValue(deviceId, out var cloud) ? cloud : null;
    }
}

public static class NativeArrayExtensions
{
    public static void DisposeIfCreated<T>(this NativeArray<T> array) where T : struct
    {
        if (array.IsCreated) array.Dispose();
    }
}