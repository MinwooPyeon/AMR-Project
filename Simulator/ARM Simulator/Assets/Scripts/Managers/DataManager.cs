using System.Collections.Generic;
using UnityEngine;
public class SensorFrame
{
    public float[] grayscaleCameraData;
    public float acceleration;
    public float chargeAmount;
    public Vector3 position;
    public AMR_STATE amrState;
    public ACTION_STATE actionState;
    public long timestamp;
}
public class DataManager
{
    private Dictionary<string, List<SensorFrame>> _deviceFrameHistory = new();

    public void AddSensorFrame(string deviceIndex, float[] cameraData, StateData state, long timestamp)
    {
        if (!_deviceFrameHistory.ContainsKey(deviceIndex))
            _deviceFrameHistory[deviceIndex] = new List<SensorFrame>();

        _deviceFrameHistory[deviceIndex].Add(new SensorFrame
        {
            grayscaleCameraData = cameraData,
            acceleration = state.Speed,
            amrState = state.AmrState,
            actionState = state.ActionState,
            position = state.transform.position,
            timestamp = timestamp
        });
    }

    public SensorFrame GetLatestFrame(string deviceIndex)
    {
        if (_deviceFrameHistory.TryGetValue(deviceIndex, out var frames) && frames.Count > 0)
            return frames[^1];
        return null;
    }

    public List<SensorFrame> GetFrameHistory(string deviceIndex)
    {
        return _deviceFrameHistory.TryGetValue(deviceIndex, out var frames) ? frames : new List<SensorFrame>();
    }

    public Dictionary<string, List<SensorFrame>> GetAllHistories()
    {
        return _deviceFrameHistory;
    }

    public void ClearAllHistories()
    {
        _deviceFrameHistory.Clear();
    }

    public class CaptureSession
    {
        public float[] cameraData = null;
        public StateData state = null;
        public long timestamp;
    }

    private Dictionary<(string deviceIndex, long timestamp), CaptureSession> _pendingSessions = new();

    public void OnCameraCaptured(string deviceIndex, float[] data, long timestamp, StateData state)
    {
        var key = (deviceIndex, timestamp);
        if (!_pendingSessions.TryGetValue(key, out var session))
        {
            session = new CaptureSession { timestamp = timestamp };
            _pendingSessions[key] = session;
        }

        session.cameraData = data;
        session.state = state;
        TryFinalizeSession(deviceIndex, session);
    }

    private void TryFinalizeSession(string deviceIndex, CaptureSession session)
    {
        if (session.cameraData != null && session.state != null)
        {
            AddSensorFrame(deviceIndex, session.cameraData, session.state, session.timestamp);
            _pendingSessions.Remove((deviceIndex, session.timestamp));
            //Debug.Log($"[DataManager] Frame saved for Device {deviceIndex}");
        }
    }
}