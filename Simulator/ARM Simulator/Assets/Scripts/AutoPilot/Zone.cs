using System.Collections.Generic;
using UnityEngine;

public class Zone : MonoBehaviour
{
    string _zoneName;
    List<Node> _nodes = new();

    public List<Node> Nodes { get { return _nodes; } }
    public string ZoneName { get { return _zoneName; } set { _zoneName = value; } }
    public void AddNode(Node node)
    {
        _nodes.Add(node);
    }

    public bool IsLocate(Node node)
    {
        return _nodes.Contains(node);
    }
}
