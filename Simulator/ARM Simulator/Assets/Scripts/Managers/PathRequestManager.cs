using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct PathRequest
{
    public Node start;
    public Node end;
    public Action<List<Node>> callback;

    public PathRequest(Node start, Node end, Action<List<Node>> callback)
    {
        this.start = start;
        this.end = end;
        this.callback = callback;
    }
}
public class PathRequestManager : MonoBehaviour
{
    private PathFinder _pathFinder = new();
    private Queue<PathRequest> _requestQueue = new();
    private bool _processing = false;
    private PathRequest _current;

    public void RequestPath(Node start, Node end, Action<List<Node>> callback)
    {
        PathRequest request = new PathRequest(start, end, callback);
        _requestQueue.Enqueue(request);
        TryProcessNext();
    }

    private void TryProcessNext()
    {
        if (_processing || _requestQueue.Count == 0) return;

        _current = _requestQueue.Dequeue();
        _processing = true;
        StartCoroutine(ProcessPath());
    }

    private IEnumerator ProcessPath()
    {
        List<Node> route = _pathFinder.PathFinding(_current.start, _current.end);
        _current.callback?.Invoke(route);
        _processing = false;
        TryProcessNext();
        yield break;
    }
}
