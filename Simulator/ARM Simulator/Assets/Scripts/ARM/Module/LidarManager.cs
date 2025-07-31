using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System.Collections.Generic;

public class LidarManager : MonoBehaviour
{
    [Header("등록된 LidarSensor들")]
    private List<LidarSensor> _sensors = new List<LidarSensor>();
    public IReadOnlyList<LidarSensor> Sensors => _sensors;

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

    public int DeviceCount => _sensors.Count;

    #region Singleton
    private static LidarManager _instance;
    public static LidarManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<LidarManager>() ?? new GameObject("LidarManager").AddComponent<LidarManager>();
                _instance.Initialize();
            }
            return _instance;
        }
    }

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject);
            Initialize();
        }
        else if (_instance != this)
        {
            Destroy(gameObject);
        }
    }

    private void Initialize()
    {
        if (!_commands.IsCreated && _sensors.Count > 0)
            RebuildArrays();
    }
    #endregion

    #region Public Registration
    public void RegisterSensor(LidarSensor sensor)
    {
        if (sensor == null || _sensors.Contains(sensor)) return;
        _sensors.Add(sensor);
        RebuildArrays();
    }

    public void UnregisterSensor(LidarSensor sensor)
    {
        if (sensor == null || !_sensors.Contains(sensor)) return;
        _sensors.Remove(sensor);
        RebuildArrays();
    }
    #endregion

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

        int sensorCount = _sensors.Count;
        AllocateSensorArrays(sensorCount);
        BuildSensorOffsetsAndCounts(sensorCount);
        BuildLocalDirections(sensorCount);
        AllocateRaycastBuffers();
    }

    private void AllocateSensorArrays(int sensorCount)
    {
        _sensorOffsets = new NativeArray<int>(sensorCount, Allocator.Persistent);
        _sensorCounts = new NativeArray<int>(sensorCount, Allocator.Persistent);
        _sensorPositions = new NativeArray<Vector3>(sensorCount, Allocator.Persistent);
        _sensorRotations = new NativeArray<Quaternion>(sensorCount, Allocator.Persistent);
        _sensorMaxDistances = new NativeArray<float>(sensorCount, Allocator.Persistent);
        _sensorLayerMasks = new NativeArray<int>(sensorCount, Allocator.Persistent);
    }

    private void BuildSensorOffsetsAndCounts(int sensorCount)
    {
        _totalRays = 0;
        for (int i = 0; i < sensorCount; i++)
        {
            var sensor = _sensors[i];
            int rayCount = sensor.localDirections.Count;

            _sensorOffsets[i] = _totalRays;
            _sensorCounts[i] = rayCount;
            _sensorMaxDistances[i] = sensor.maxDistance;
            _sensorLayerMasks[i] = sensor.layerMask;

            _totalRays += rayCount;
        }
    }

    private void BuildLocalDirections(int sensorCount)
    {
        _localDirections = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
        int index = 0;
        foreach (var sensor in _sensors)
        {
            foreach (var dir in sensor.localDirections)
                _localDirections[index++] = dir;
        }
    }

    private void AllocateRaycastBuffers()
    {
        _commands = new NativeArray<RaycastCommand>(_totalRays, Allocator.Persistent);
        _points = new NativeArray<RaycastHit>(_totalRays, Allocator.Persistent);
        _outPoints = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
    }

    public void ScanDevice(int index, long timestamp)
    {
        if (index < 0 || index >= _sensors.Count) return;

        var sensor = _sensors[index];
        int offset = _sensorOffsets[index];
        int count = _sensorCounts[index];

        _sensorPositions[index] = sensor.transform.position;
        _sensorRotations[index] = sensor.transform.rotation;

        var tHandle = ScheduleTransformJob();
        var rHandle = ScheduleRaycastJob(offset, count, tHandle);
        var dHandle = ScheduleDistributeJob(offset, count, rHandle);

        dHandle.Complete();
        WriteBackSensorResults(sensor, offset, count, timestamp);
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

    private JobHandle ScheduleDistributeJob(int count, int offset, JobHandle dependency)
    {
        var dJob = new DistributeJob
        {
            points = _points,
            commands = _commands,
            outPoints = _outPoints
        };
        return dJob.Schedule(count, 64, dependency);
    }

    private void WriteBackSensorResults(LidarSensor sensor, int offset, int count, long timestamp)
    {
        for (int i = 0; i < count; i++)
            sensor.pointCloud[i] = _outPoints[offset + i];

        Debug.Log($"[Lidar:{sensor.name}] Scanned {count} points at {timestamp}");
        // TODO: Tensor 변환, 전송 등 후처리 (timestamp 포함)
    }
}

public static class NativeArrayExtensions
{
    public static void DisposeIfCreated<T>(this NativeArray<T> array) where T : struct
    {
        if (array.IsCreated) array.Dispose();
    }
}
