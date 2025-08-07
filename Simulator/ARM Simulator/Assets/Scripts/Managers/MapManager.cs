using System.Collections.Generic;
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
}
