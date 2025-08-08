using UnityEngine;

public class VirtualNavigator : MonoBehaviour
{
    public StateData _state;
    public VirtualDeviceController _deviceController;

    private Node _start;
    private Node _end;
    


    private void Start()
    {
        AssignTask();
    }

    public void AssignTask()
    {
        _end = GetEndPosition();
        RequestAndMove();
    }

    private Node GetEndPosition()
    {
        switch (_state.AmrState)
        {
            case AMR_STATE.RUNNING:
            case AMR_STATE.DROP:
                return Managers.Map.GetRandomLoaderPos();
            case AMR_STATE.LOAD:
                return Managers.Map.GetRandomDroperPos();
            default:
                return null;
        }
    }

    private void RequestAndMove()
    {
        _start = Managers.Map.Grid.GetLocateNode((int)_state.GridPosition.x, (int)_state.GridPosition.y);
        Managers.Path.RequestPath(_start, _end, route =>
        {
            if (route == null || route.Count == 0)
            {
                //AssignTask();
            }
            else
            {
                _deviceController.MoveDevice(route, AssignTask);
            }
        });
    }
}
