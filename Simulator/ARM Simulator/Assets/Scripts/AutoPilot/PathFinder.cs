using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A* PathFinder
/// - ����(step) ���: 1
/// - ��(���� ��ȯ) ���: 3
/// - �޸���ƽ: ����ư �Ÿ�(4/8���� ��� ����)
/// </summary>
public class PathFinder
{
    private Node _start;
    private Node _current;
    private Node _end;

    private readonly List<Node> _candidate = new();
    private readonly List<Node> _complete = new();

    public Node Start => _start;
    public Node End => _end;
    public Node Current => _current;

    /// <summary>
    /// �ĺ�/�Ϸ� ��Ͽ� ����ִ� ������ �ڽ�Ʈ�� �ʱ�ȭ�մϴ�.
    /// (����: ���� ��� ��带 �ʱ�ȭ�ϴ� ��ƿ�� ������ �θ� �� ����)
    /// </summary>
    private void ResetCost()
    {
        for (int i = 0; i < _candidate.Count; i++)
        {
            _candidate[i].GCost = int.MaxValue;
            _candidate[i].Parent = null;
            _candidate[i].HCost = 0;
        }

        for (int i = 0; i < _complete.Count; i++)
        {
            _complete[i].GCost = int.MaxValue;
            _complete[i].Parent = null;
            _complete[i].HCost = 0;
        }
    }

    private void ClearList()
    {
        _candidate.Clear();
        _complete.Clear();
    }

    /// <summary>
    /// ���� �̵� ���Ϳ� �̹� �̵� ���Ͱ� �ٸ��� '��'���� �Ǵ��Ͽ� ��� 3, ���� �����̸� 1 ��ȯ
    /// ù ����(from.Parent == null)�� ������ 1
    /// </summary>
    private static int StepCost(Node from, Node to)
    {
        if (from.Parent == null) return 1;

        var prevDir = from.Pos - from.Parent.Pos; // ���� �̵� ����
        var newDir = to.Pos - from.Pos;        // �̹� �̵� ����

        return (prevDir != newDir) ? 3 : 1;
    }

    /// <summary>
    /// �̿� ����� �ڽ�Ʈ�� ����/����
    /// </summary>
    private void CalcCost(Node neighbor)
    {
        // �̹� close set(�Ϸ�)�� ��ŵ
        if (_complete.Contains(neighbor)) return;

        int step = StepCost(_current, neighbor);
        float tentativeG = _current.GCost + step;

        if (tentativeG < neighbor.GCost)
        {
            neighbor.Parent = _current;
            neighbor.GCost = tentativeG;

            // H�� 'neighbor' ������ ����ư �Ÿ�
            neighbor.HCost = Mathf.Abs(_end.Pos.x - neighbor.Pos.x) + Mathf.Abs(_end.Pos.y - neighbor.Pos.y);

            // open set(�ĺ�)�� ���ٸ� �߰�
            if (!_candidate.Contains(neighbor))
                _candidate.Add(neighbor);
        }
    }

    /// <summary>
    /// FCost(=G+H) ��������, �����̸� HCost ���� ��
    /// </summary>
    private static int SortCost(Node a, Node b)
    {
        if (a.FCost == b.FCost) return a.HCost.CompareTo(b.HCost);
        return a.FCost.CompareTo(b.FCost);
    }

    /// <summary>
    /// ���� Ž�� ��� ����
    /// </summary>
    private void SelectNextNode()
    {
        // ���� ���� close set���� �̵�
        _candidate.Remove(_current);
        _complete.Add(_current);

        if (_candidate.Count == 0)
        {
            Debug.LogError("PathFinder: ��θ� ã�� �� �����ϴ�. (open set empty)");
            _current = null;
            return;
        }

        _candidate.Sort(SortCost);
        _current = _candidate[0];
    }

    /// <summary>
    /// ���� ����� �̿��� ��
    /// </summary>
    private void EvaluateNeighbors(Node node)
    {
        List<Node> neighbors = Managers.Map.Grid.GetNeighborNode(node);
        for (int i = 0; i < neighbors.Count; i++)
            CalcCost(neighbors[i]);
    }

    /// <summary>
    /// _end���� Parent�� Ÿ�� �ö� ��θ� �籸��
    /// </summary>
    private List<Node> ReconstructPath()
    {
        var path = new List<Node>();
        var visited = new HashSet<Node>();
        Node node = _end;

        while (node != null)
        {
            // ����Ŭ ����
            if (!visited.Add(node))
            {
                Debug.LogError("PathFinder.GetRoute: Parent ü�ο� ����Ŭ�� �����Ǿ����ϴ�.");
                break;
            }

            path.Add(node);
            if (node == _start) break;
            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    /// <summary>
    /// �ܺ� ȣ�� ������
    /// </summary>
    public List<Node> PathFinding(Node start, Node end)
    {
        ResetCost();
        ClearList();

        _start = start;
        _end = end;
        _current = start;

        // ���� ��� �ʱ�ȭ
        _start.Parent = null;
        _start.GCost = 0;
        _start.HCost = Mathf.Abs(_end.Pos.x - _start.Pos.x) + Mathf.Abs(_end.Pos.y - _start.Pos.y);

        // ���� ��带 open set�� �ְ� ����
        _candidate.Add(_start);

        while (_current != null && _current != _end)
        {
            EvaluateNeighbors(_current);
            SelectNextNode(); // open set���� ���� �ĺ� ������
        }

        if (_current == null)
            return null; // ��� ����

        // ���� ��� close set �̵�(����)
        _candidate.Remove(_current);
        _complete.Add(_current);

        return ReconstructPath();
    }
}