using Unity.VisualScripting;
using UnityEngine;

public class LoaderNode : Node
{
    public override void SetData(Vector2Int pos)
    {
        Walkable = true;
        Pos = pos;
        NodeType = NODE_TYPE.LOADER;
        Parent = null;
    }
}
