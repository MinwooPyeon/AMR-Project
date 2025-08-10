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
            node.GCost = cost + _current.GCost;
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
        var path = new List<Node>();
        var visited = new HashSet<Node>();
        Node node = _end;

        while (node != null)
        {
            // 사이클 체크: 이미 본 노드면 경고하고 빠져나감
            if (!visited.Add(node))
            {
                Debug.LogError("GetRoute: Parent 체인에 사이클이 감지되었습니다.");
                break;
            }

            path.Add(node);

            // 시작점에 도달했으면 정상 종료
            if (node == _start)
                break;

            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    public List<Node> PathFinding(Node start, Node end)
    {
        //Debug.Log($"{start.Pos}, {end.Pos}");
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
