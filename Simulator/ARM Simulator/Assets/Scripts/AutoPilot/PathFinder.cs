using System.Collections.Generic;
using UnityEngine;

public class PathFinder
{
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
            _candidate[idx].GCost = 9999999;
            _candidate[idx].Parent = null;
        }

        maxIdx = _complete.Count;
        for (int idx = 0; idx < maxIdx; idx++)
        {
            _complete[idx].GCost = 9999999;
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
        if (cost + _current.GCost < node.GCost || node.Parent == null)
        {
            node.Parent = _current;
            node.GCost = cost + _current.HCost;
            node.HCost = Mathf.Abs(_end.Pos.x - _current.Pos.x) + Mathf.Abs(_end.Pos.y - _current.Pos.y);
        }
    }
    private int SortCost(Node anode, Node bnode)
    {
        return anode.FCost == bnode.FCost ? anode.HCost.CompareTo(bnode.HCost) : anode.FCost.CompareTo(bnode.FCost);
    }
    private void SelectNextNode()
    {
        _candidate.Remove(_current);
        _complete.Add(_current);
        _candidate.Sort(SortCost);
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
        //Debug.Log(node.Pos);
        List<Node> nodes = Managers.Map.Grid.GetNeighborNode(node);
        for (int i = 0; i < nodes.Count; i++)
        {
            CalcCost(nodes[i]);
            if (!_complete.Contains(nodes[i]) && !_candidate.Contains(nodes[i])){
                _candidate.Add(nodes[i]);
            }
        }
    }

    private List<Node> GetRoute()
    {
        Node idx = _end;
        List<Node> routes = new();

        while (idx != null) 
        {
            routes.Add(idx);
            if (idx == _start) break;
            idx = idx.Parent;
        }
        routes.Reverse();
        return routes;
    }
    public List<Node> PathFinding(Node start, Node end)
    {
        ResetCost();
        ClearList();

        _start = start;
        _end = end;
        _current = start;

        while(_current != _end)
        {
            GetCandidateNode(_current);
            SelectNextNode();
            if (_current == null) return null;
        }
        _candidate.Remove(_current);
        _complete.Add(_current);
        return GetRoute();
    }
}
