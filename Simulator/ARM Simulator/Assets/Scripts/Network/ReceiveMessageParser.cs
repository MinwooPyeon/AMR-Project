using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class ReceiveMessageParser
{
    JsonParser parser = new();
    
    public MapMsg ParseMapMessage(string json)
    {
        MapMsg msg = new MapMsg();

        Dictionary<string, string> pairs = parser.Parse(json);

        

        return msg;
    }

    public StatusMsg ParseStatusMessage(string json)
    {
        StatusMsg msg = new StatusMsg();

        Dictionary<string, string> pairs = parser.Parse(json);

        msg.serialNumber = pairs["serial"];
        msg.state = (AMR_STATE)Enum.Parse(typeof(AMR_STATE), pairs["state"], true);
        msg.position.x = float.Parse(pairs["x"]);
        msg.position.y = float.Parse(pairs["y"]);
        msg.speed = float.Parse(pairs["speed"]);
        msg.angle = float.Parse(pairs["angle"]);
        return msg;
    }
}

//Json Parsing Output Class
public class MapMsg
{
    public string map;
    public Texture2D texture;
}

public class StatusMsg
{
    public string serialNumber;
    public AMR_STATE state;
    public Vector2 position;
    public float speed;
    public float angle;
}