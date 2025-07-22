using System;
using System.Collections.Generic;
using UnityEngine;

public class DeviceController : MonoBehaviour
{
    #region Attribute
    MoveController _moveController;
    Dictionary<AIORDER, Action<float>> _commands;
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
    private void Awake()
    {
        _moveController = GetComponent<MoveController>();

        // 명령어 초기화
        _commands = new Dictionary<AIORDER, Action<float>>
        {
            { AIORDER.MOVE_FORWARD, _moveController.MoveForward },
            { AIORDER.STOP, _moveController.Stop },
            { AIORDER.ROTATE_LEFT, _ => _moveController.RotateLeft() },
            { AIORDER.ROTATE_RIGHT, _ => _moveController.RotateRight() }
        };

        // DeviceManager에 등록
        Managers.Device.DeviceRegister(GetInstanceID(), this);
    }

    private void OnDestroy()
    {
        Managers.Device.DeviceUnregister(GetInstanceID());
    }
    #endregion
}
