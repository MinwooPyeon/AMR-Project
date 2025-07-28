using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using System.Collections.Generic;
using Unity.Collections.LowLevel.Unsafe;

public class LidarManager : MonoBehaviour
{
    [Header("등록된 LidarSensor들")]
    private List<LidarSensor> _sensors = new List<LidarSensor>();

    [Header("스캔 주기 (초)")]
    public float scanInterval = 0.1f;

    // 외부에서 센서를 조회해야 할 경우, 수정되지 않도록 IReadOnlyList로 제공
    public IReadOnlyList<LidarSensor> Sensors => _sensors;

    // Flattened sensor data
    private NativeArray<int> _sensorOffsets;
    private NativeArray<int> _sensorCounts;
    private NativeArray<Vector3> _localDirections;
    private NativeArray<Vector3> _sensorPositions;
    private NativeArray<Quaternion> _sensorRotations;
    private NativeArray<float> _sensorMaxDistances;
    private NativeArray<int> _sensorLayerMasks;

    // Job buffers
    private NativeArray<RaycastCommand> _commands;
    private NativeArray<RaycastHit> _points;
    private NativeArray<Vector3> _outPoints;

    private int _totalRays;
    private float _timer;

    #region Singleton
    private static LidarManager _instance;
    public static LidarManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<LidarManager>();
                if (_instance == null)
                {
                    var go = new GameObject("LidarManager");
                    _instance = go.AddComponent<LidarManager>();
                }
                _instance.Initialize();
            }
            return _instance;
        }
    }

    private void Initialize()
    {
        if (!_commands.IsCreated && _sensors.Count > 0)
            RebuildArrays();
    }
    #endregion

    void Awake()
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
            return;
        }
    }

    void Update()
    {
        if (_sensors.Count == 0) return;

        _timer -= Time.deltaTime;
        if (_timer > 0f) return;
        _timer += scanInterval;

        // 1) Update sensor transforms
        for (int i = 0; i < _sensorPositions.Length; i++)
        {
            _sensorPositions[i] = _sensors[i].transform.position;
            _sensorRotations[i] = _sensors[i].transform.rotation;
        }

        // 2) TransformJob: build RaycastCommands
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
        JobHandle tHandle = tJob.Schedule(_sensorOffsets.Length, 1);

        // 3) Raycast schedule
        int cores = SystemInfo.processorCount;
        int batchSize = Mathf.Max(1, _totalRays / (cores * 4));
        JobHandle rHandle = RaycastCommand.ScheduleBatch(_commands, _points, batchSize, tHandle);

        // 4) DistributeJob: compute final hit points
        var dJob = new DistributeJob
        {
            points = _points,
            commands = _commands,
            outPoints = _outPoints
        };
        JobHandle dHandle = dJob.Schedule(_totalRays, 64, rHandle);

        // 5) Complete and write back
        dHandle.Complete();
        CopyResultsToSensors();
    }

    void OnDestroy()
    {
        // Dispose all allocated arrays
        if (_commands.IsCreated) _commands.Dispose();
        if (_points.IsCreated) _points.Dispose();
        if (_outPoints.IsCreated) _outPoints.Dispose();
        if (_sensorOffsets.IsCreated) _sensorOffsets.Dispose();
        if (_sensorCounts.IsCreated) _sensorCounts.Dispose();
        if (_localDirections.IsCreated) _localDirections.Dispose();
        if (_sensorPositions.IsCreated) _sensorPositions.Dispose();
        if (_sensorRotations.IsCreated) _sensorRotations.Dispose();
        if (_sensorMaxDistances.IsCreated) _sensorMaxDistances.Dispose();
        if (_sensorLayerMasks.IsCreated) _sensorLayerMasks.Dispose();
    }

    #region Public Registration
    /// <summary>
    /// 외부에서 LidarSensor를 등록합니다.
    /// </summary>
    public void RegisterSensor(LidarSensor sensor)
    {
        if (sensor == null || _sensors.Contains(sensor)) return;
        _sensors.Add(sensor);
        RebuildArrays();
    }

    /// <summary>
    /// 외부에서 LidarSensor를 제거합니다.
    /// </summary>
    public void UnregisterSensor(LidarSensor sensor)
    {
        if (sensor == null || !_sensors.Contains(sensor)) return;
        _sensors.Remove(sensor);
        RebuildArrays();
    }
    #endregion

    private void RebuildArrays()
    {
        // Dispose existing
        if (_commands.IsCreated) _commands.Dispose();
        if (_points.IsCreated) _points.Dispose();
        if (_outPoints.IsCreated) _outPoints.Dispose();
        if (_sensorOffsets.IsCreated) _sensorOffsets.Dispose();
        if (_sensorCounts.IsCreated) _sensorCounts.Dispose();
        if (_localDirections.IsCreated) _localDirections.Dispose();
        if (_sensorPositions.IsCreated) _sensorPositions.Dispose();
        if (_sensorRotations.IsCreated) _sensorRotations.Dispose();
        if (_sensorMaxDistances.IsCreated) _sensorMaxDistances.Dispose();
        if (_sensorLayerMasks.IsCreated) _sensorLayerMasks.Dispose();

        int sensorCount = _sensors.Count;
        _sensorOffsets = new NativeArray<int>(sensorCount, Allocator.Persistent);
        _sensorCounts = new NativeArray<int>(sensorCount, Allocator.Persistent);
        _sensorPositions = new NativeArray<Vector3>(sensorCount, Allocator.Persistent);
        _sensorRotations = new NativeArray<Quaternion>(sensorCount, Allocator.Persistent);
        _sensorMaxDistances = new NativeArray<float>(sensorCount, Allocator.Persistent);
        _sensorLayerMasks = new NativeArray<int>(sensorCount, Allocator.Persistent);

        _totalRays = 0;
        for (int i = 0; i < sensorCount; i++)
        {
            var s = _sensors[i];
            int count = s.localDirections.Count;
            _sensorOffsets[i] = _totalRays;
            _sensorCounts[i] = count;
            _totalRays += count;
            _sensorMaxDistances[i] = s.maxDistance;
            _sensorLayerMasks[i] = s.layerMask;
        }

        _localDirections = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
        int idx = 0;
        for (int i = 0; i < sensorCount; i++)
        {
            var dirs = _sensors[i].localDirections;
            for (int j = 0; j < dirs.Count; j++)
                _localDirections[idx++] = dirs[j];
        }

        _commands = new NativeArray<RaycastCommand>(_totalRays, Allocator.Persistent);
        _points = new NativeArray<RaycastHit>(_totalRays, Allocator.Persistent);
        _outPoints = new NativeArray<Vector3>(_totalRays, Allocator.Persistent);
    }

    private void CopyResultsToSensors()
    {
        int idx = 0;
        for (int si = 0; si < _sensors.Count; si++)
        {
            var sensor = _sensors[si];
            int count = _sensorCounts[si];
            for (int i = 0; i < count; i++)
                sensor.pointCloud[i] = _outPoints[idx + i];
            Debug.Log("[LidarManager] Sensor " + si + " updated with " + count + " points.");
            idx += count;
        }
    }

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
            if (hit.distance > 0f)
                outPoints[i] = hit.point;
            else
            {
                var cmd = commands[i];
                outPoints[i] = cmd.from + cmd.direction * cmd.distance;
            }
        }
    }
}
