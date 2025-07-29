using System.Collections.Generic;
using UnityEngine;

public class DataManager : MonoBehaviour
{
    [System.Serializable]
    public class SensorFrame
    {
        public float[] grayscaleCameraData;
        public Vector3[] lidarPoints;
        public float acceleration;
        public float chargeAmount;
        public ARM_STATE armState;
        public ACTION_STATE actionState;
        public long timestamp;
    }

    private Dictionary<int, SensorFrame> _latestFrames = new();

    public void UpdateSensorFrame(int deviceIndex, float[] cameraData, Vector3[] lidarData, StateData state, long timestamp)
    {
        _latestFrames[deviceIndex] = new SensorFrame
        {
            grayscaleCameraData = cameraData,
            lidarPoints = lidarData,
            acceleration = state.Acceleration,
            chargeAmount = state.ChargeAmount,
            armState = state.ArmState,
            actionState = state.ActionState,
            timestamp = timestamp
        };
    }

    public SensorFrame GetFrame(int deviceIndex)
    {
        return _latestFrames.TryGetValue(deviceIndex, out var frame) ? frame : null;
    }

    public List<SensorFrame> GetAllFrames()
    {
        return new List<SensorFrame>(_latestFrames.Values);
    }

    public void ClearAllFrames()
    {
        _latestFrames.Clear();
    }
}
