using System;
using System.Collections.Generic;
using UnityEngine;

public class DeviceController : MonoBehaviour
{
    #region Attribute
    MoveController _moveController;
    Dictionary<ACTION_STATE, Action<float>> _commands;
    #endregion
    #region Methods
    public void ExcuteOrder(ActionOrder order)
    {
        if (_commands.TryGetValue(order.Order, out var action))
        {
            action(order.Param);
        }
    }
    #endregion
    #region Unity Methods
    private void Start()
    {
        _moveController = GetComponent<MoveController>();

        // 명령어 초기화
        _commands = new Dictionary<ACTION_STATE, Action<float>>
        {
            { ACTION_STATE.MOVE_FORWARD, _moveController.MoveForward },
            { ACTION_STATE.MOVE_BACKWARD, _moveController.MoveBackward },
            { ACTION_STATE.STOP, _moveController.Stop },
            { ACTION_STATE.ROTATE_LEFT, _ => _moveController.RotateLeft() },
            { ACTION_STATE.ROTATE_RIGHT, _ => _moveController.RotateRight() }
        };

        // DeviceManager에 등록
        //Managers.Device.RegistVirtualDevice(this.gameObject.GetInstanceID().ToString(), this);
    }

    private void OnDestroy()
    {
        Managers.Device.DeviceUnregister(this.gameObject.GetInstanceID().ToString());
    }
    #endregion
}
