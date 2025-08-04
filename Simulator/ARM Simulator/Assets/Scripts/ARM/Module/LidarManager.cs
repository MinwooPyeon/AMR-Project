using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Rendering;
using System.Collections.Generic;
using System.Linq;

public class LidarManager : MonoBehaviour
{
    // ──────────────────────────────────────────────
    private Dictionary<int, LidarSensor> _sensorMap = new();
    public IReadOnlyDictionary<int, LidarSensor> Sensors => _sensorMap;

    // 풀링용 NativeArray 사전
    private Dictionary<int, NativeArray<RaycastCommand>> _commandPool = new();
    private Dictionary<int, NativeArray<RaycastHit>> _hitPool = new();

    // 비동기 스캔 완료 대기 리스트
    struct PendingScan
    {
        public int deviceId;
        public long timestamp;
        public JobHandle handle;
    }
    private List<PendingScan> _pendingScans = new();

    // ──────────────────────────────────────────────
    private void Update()
    {
        // 완료된 스캔만 처리
        for (int i = _pendingScans.Count - 1; i >= 0; --i)
        {
            var scan = _pendingScans[i];
            if (scan.handle.IsCompleted)
            {
                scan.handle.Complete();
                ProcessScanResults(scan.deviceId, scan.timestamp);
                _pendingScans.RemoveAt(i);
            }
        }
    }

    private void OnDestroy()
    {
        // 모든 풀 해제
        foreach (var arr in _commandPool.Values) arr.Dispose();
        foreach (var arr in _hitPool.Values) arr.Dispose();
        _commandPool.Clear();
        _hitPool.Clear();
    }

    // ──────────────────────────────────────────────
    public void RegisterSensor(int deviceId, LidarSensor sensor)
    {
        if (_sensorMap.ContainsKey(deviceId)) return;
        _sensorMap[deviceId] = sensor;

        int count = sensor.localDirections.Count;
        if (count > 0)
        {
            _commandPool[deviceId] = new NativeArray<RaycastCommand>(count, Allocator.Persistent);
            _hitPool[deviceId] = new NativeArray<RaycastHit>(count, Allocator.Persistent);
        }
    }

    public void UnregisterSensor(int deviceId)
    {
        if (!_sensorMap.Remove(deviceId)) return;
        if (_commandPool.TryGetValue(deviceId, out var cmdArr))
        {
            cmdArr.Dispose();
            _commandPool.Remove(deviceId);
        }
        if (_hitPool.TryGetValue(deviceId, out var hitArr))
        {
            hitArr.Dispose();
            _hitPool.Remove(deviceId);
        }
    }

    // ──────────────────────────────────────────────
    public void RequestScan(int deviceId, long timestamp)
    {
        // 1) 이전에 같은 deviceId로 스케줄된 Job이 있으면 먼저 완료하고 처리
        for (int i = _pendingScans.Count - 1; i >= 0; --i)
        {
            if (_pendingScans[i].deviceId == deviceId)
            {
                var prev = _pendingScans[i];
                prev.handle.Complete();
                ProcessScanResults(prev.deviceId, prev.timestamp);
                _pendingScans.RemoveAt(i);
            }
        }

        // 2) 풀 준비 확인
        if (!_sensorMap.TryGetValue(deviceId, out var sensor)) return;
        if (!_commandPool.ContainsKey(deviceId)) return;

        var dirs = sensor.localDirections;
        var commands = _commandPool[deviceId];
        var hits = _hitPool[deviceId];
        int count = dirs.Count;
        if (count == 0) return;

        // 3) RaycastCommand 생성
        Vector3 origin = sensor.transform.parent.position;
        Quaternion rot = sensor.transform.parent.rotation;
        float maxDist = sensor.maxDistance;
        int layerMask = sensor.layerMask;

        for (int i = 0; i < count; i++)
        {
            Vector3 dirWorld = rot * dirs[i];
            commands[i] = new RaycastCommand(origin, dirWorld, maxDist, layerMask);
        }

        // 4) 배치 스케줄
        int batchSize = Mathf.Max(1, count / (SystemInfo.processorCount * 4));
        JobHandle handle = RaycastCommand.ScheduleBatch(commands, hits, batchSize, default);

        // 5) 완료 대기 큐에 추가
        _pendingScans.Add(new PendingScan
        {
            deviceId = deviceId,
            timestamp = timestamp,
            handle = handle
        });
    }

    // ──────────────────────────────────────────────
    private void ProcessScanResults(int deviceId, long timestamp)
    {
        var sensor = _sensorMap[deviceId];
        var commands = _commandPool[deviceId];
        var hits = _hitPool[deviceId];
        int count = commands.Length;

        var points = new Vector3[count];
        for (int i = 0; i < count; i++)
        {
            var hit = hits[i];
            if (hit.collider != null)
                points[i] = hit.point;
            else
            {
                var cmd = commands[i];
                points[i] = cmd.from + cmd.direction.normalized * cmd.distance;
            }
        }

        sensor.pointCloud = points;
        var go = sensor.transform.parent.gameObject;
        var state = go.GetComponent<StateData>();
        Managers.Data.OnLidarScanned(deviceId, points, timestamp, state);
        Debug.Log("[Lidar Manager]Lidar Complete");

        //SLAM
        OccupancyGridManager.Instance.AddLidarPoints(points, sensor);
    }
}
