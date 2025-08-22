using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A* PathFinder
/// - 직진(step) 비용: 1
/// - 턴(방향 전환) 비용: 3
/// - 휴리스틱: 맨해튼 거리(4/8방향 모두 안전)
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
    /// 후보/완료 목록에 들어있는 노드들의 코스트만 초기화합니다.
    /// (권장: 맵의 모든 노드를 초기화하는 유틸을 별도로 두면 더 안전)
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
    /// 직전 이동 벡터와 이번 이동 벡터가 다르면 '턴'으로 판단하여 비용 3, 동일 방향이면 1 반환
    /// 첫 스텝(from.Parent == null)은 무조건 1
    /// </summary>
    private static int StepCost(Node from, Node to)
    {
        if (from.Parent == null) return 1;

        var prevDir = from.Pos - from.Parent.Pos; // 이전 이동 벡터
        var newDir = to.Pos - from.Pos;        // 이번 이동 벡터

        return (prevDir != newDir) ? 3 : 1;
    }

    /// <summary>
    /// 이웃 노드의 코스트를 재평가/갱신
    /// </summary>
    private void CalcCost(Node neighbor)
    {
        // 이미 close set(완료)면 스킵
        if (_complete.Contains(neighbor)) return;

        int step = StepCost(_current, neighbor);
        float tentativeG = _current.GCost + step;

        if (tentativeG < neighbor.GCost)
        {
            neighbor.Parent = _current;
            neighbor.GCost = tentativeG;

            // H는 'neighbor' 기준의 맨해튼 거리
            neighbor.HCost = Mathf.Abs(_end.Pos.x - neighbor.Pos.x) + Mathf.Abs(_end.Pos.y - neighbor.Pos.y);

            // open set(후보)에 없다면 추가
            if (!_candidate.Contains(neighbor))
                _candidate.Add(neighbor);
        }
    }

    /// <summary>
    /// FCost(=G+H) 오름차순, 동률이면 HCost 작은 순
    /// </summary>
    private static int SortCost(Node a, Node b)
    {
        if (a.FCost == b.FCost) return a.HCost.CompareTo(b.HCost);
        return a.FCost.CompareTo(b.FCost);
    }

    /// <summary>
    /// 다음 탐색 노드 선택
    /// </summary>
    private void SelectNextNode()
    {
        // 현재 노드는 close set으로 이동
        _candidate.Remove(_current);
        _complete.Add(_current);

        if (_candidate.Count == 0)
        {
            Debug.LogError("PathFinder: 경로를 찾을 수 없습니다. (open set empty)");
            _current = null;
            return;
        }

        _candidate.Sort(SortCost);
        _current = _candidate[0];
    }

    /// <summary>
    /// 현재 노드의 이웃을 평가
    /// </summary>
    private void EvaluateNeighbors(Node node)
    {
        List<Node> neighbors = Managers.Map.Grid.GetNeighborNode(node);
        for (int i = 0; i < neighbors.Count; i++)
            CalcCost(neighbors[i]);
    }

    /// <summary>
    /// _end부터 Parent를 타고 올라가 경로를 재구성
    /// </summary>
    private List<Node> ReconstructPath()
    {
        var path = new List<Node>();
        var visited = new HashSet<Node>();
        Node node = _end;

        while (node != null)
        {
            // 사이클 방지
            if (!visited.Add(node))
            {
                Debug.LogError("PathFinder.GetRoute: Parent 체인에 사이클이 감지되었습니다.");
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
    /// 외부 호출 진입점
    /// </summary>
    public List<Node> PathFinding(Node start, Node end)
    {
        ResetCost();
        ClearList();

        _start = start;
        _end = end;
        _current = start;

        // 시작 노드 초기화
        _start.Parent = null;
        _start.GCost = 0;
        _start.HCost = Mathf.Abs(_end.Pos.x - _start.Pos.x) + Mathf.Abs(_end.Pos.y - _start.Pos.y);

        // 시작 노드를 open set에 넣고 시작
        _candidate.Add(_start);

        while (_current != null && _current != _end)
        {
            EvaluateNeighbors(_current);
            SelectNextNode(); // open set에서 최적 후보 꺼내기
        }

        if (_current == null)
            return null; // 경로 없음

        // 도착 노드 close set 이동(선택)
        _candidate.Remove(_current);
        _complete.Add(_current);

        return ReconstructPath();
    }
}