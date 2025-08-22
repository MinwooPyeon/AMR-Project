
public enum AMR_STATE
{
    RUNNING,
    RECHARGE,
    DROP,
    LOAD,
}
public enum  ACTION_STATE
{
    MOVE_FORWARD,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    MOVE_BACKWARD,
    LIFT,
    STOP
}

public enum DangerCase 
{
    NO_HELMET,
    BOX_COLLISION
}
public enum BrushColor
{
    White, Black, Red, Blue, Green, Yellow, Gray
}

public enum NODE_TYPE
{
    FREE,
    OBSTACLE,
    CHARGER,
    LOADER,
    DROPER
}