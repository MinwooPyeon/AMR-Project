using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class MapManager
{
    List<Vector3> _chargers = new();
    List<Vector3> _loaders = new();
    List<Vector3> _dropers = new();
    Grid grid = new();
    
    public List<Vector3> Chargers {  get { return _chargers; } }
    public List<Vector3> Loaders { get {return _loaders; } }
    public List<Vector3> Dropers { get {return _dropers; } }
    public Grid Grid { get { return grid; } }
    
    public void AddChargerPos(Vector3 pos)
    {
        _chargers.Add(pos);
    }
    public void AddLoaderPos(Vector3 pos)
    {
        _loaders.Add(pos);
    }
    public void AddDroperPos(Vector3 pos)
    {
        _dropers.Add(pos);
    }
    public void ClearPoses()
    {
        _chargers.Clear();
        _loaders.Clear();
        _dropers.Clear();
    }

    public Node GetRandomChargerPos()
    {
        if (_chargers.Count == 0) throw new System.InvalidOperationException("No chargers registered");
        int idx = Random.Range(0, _chargers.Count);
        Vector3 charger = _chargers[idx];
        return Grid.GetLocateNode((int)charger.x, (int)charger.z);
    }

    public Node GetRandomLoaderPos()
    {
        if (_loaders.Count == 0) throw new System.InvalidOperationException("No loaders registered");
        int idx = Random.Range(0, _loaders.Count);
        Vector3 loader = _loaders[idx];
        return Grid.GetLocateNode((int)loader.x, (int)loader.z);
    }
    public Node GetRandomDroperPos()
    {
        if (_dropers.Count == 0) throw new System.InvalidOperationException("No dropers registered");
        int idx = Random.Range(0, _dropers.Count);
        Vector3 droper = _dropers[idx];
        return Grid.GetLocateNode((int)droper.x, (int)droper.z);
    }
}
