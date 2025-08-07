using System.Collections.Generic;
using System.Xml.Serialization;
using UnityEngine;

public class PathFinder : MonoBehaviour
{
    private bool _isFinish;

    private Node _start;
    private Node _current;
    private Node _end;

    List<Node> _candidate = new();
    List<Node> _complete = new();

    public Node Start { get { return _start; } }
    public Node End { get { return _end; } }
    public Node Current { get { return _current; } }

    private void ResetCost()
    {
        int maxIdx = _candidate.Count;
        for (int idx = 0; idx < maxIdx; idx++)
        {
            _candidate[idx].HCost = 0;
            _candidate[idx].Parent = null;
        }

        maxIdx = _complete.Count;
        for (int idx = 0; idx < maxIdx; idx++)
        {
            _complete[idx].HCost = 0;
            _complete[idx].Parent = null;
        }
    }
    private void ClearList()
    {
        _candidate.Clear();
        _complete.Clear();
    }
    private void CalcCost(Node node)
    {
        int cost = 1;
        if (cost + _current.HCost < node.HCost || node.Parent == null)
        {
            node.Parent = _current;
            node.HCost = cost + _current.HCost;
        }
    }
    private int SortHCost(Node anode, Node bnode)
    {
        return anode.HCost.CompareTo(bnode.HCost);
    }
    private void SelectNextNode()
    {
        _candidate.Remove(_current);
        _complete.Add(_current);
        _candidate.Sort(SortHCost);
        if (_candidate.Count <= 0)
        {
            Debug.Log("Error : Cant Find Path");
            _current = null;
            return;
        }
        _current = _candidate[0];
    }
    private void GetCandidateNode(Node node)
    {
        List<Node> nodes = Managers.Map.Grid.GetNeighborNode(node);
        for (int i = 0; i < nodes.Count; i++)
        {
            if (CheckEnd(nodes[i])) return;
            CalcCost(nodes[i]);
            if (!_complete.Contains(nodes[i]) && !_candidate.Contains(nodes[i]){
                _candidate.Add(nodes[i]);
            }
        }
    }
    private bool CheckEnd(Node node)
    {
        if (node == _end) { 
            _end = _current;
            _isFinish = true;
            return true; 
        }
        _isFinish = false;
        return false;
    }

    private List<Node> GetRoute()
    {
        Node idx = _end;
        List<Node> routes = new();

        while (idx != null) 
        {
            routes.Add(idx);
            idx = idx.Parent;
        }

        return routes;
    }
    public List<Node> PathFinding(Node start, Node end)
    {
        ResetCost();
        ClearList();

        _start = start;
        _end = end;
        _current = start;

        while(!_isFinish)
        {
            GetCandidateNode(_current);
            SelectNextNode();
            if (_current == null) return null;
        }

        return GetRoute();
    }
}
