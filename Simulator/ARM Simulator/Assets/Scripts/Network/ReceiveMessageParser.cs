using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class ReceiveMessageParser
{
    JsonParser parser = new();
    
    public PositionMsg ParsePositionMessage(string json)
    {
        PositionMsg msg = new PositionMsg();

        Dictionary<string, string> pairs = parser.Parse(json);

        msg.actionState = (ACTION_STATE)Enum.Parse(typeof(ACTION_STATE) ,pairs["move"], true);
        msg.position.x = float.Parse(pairs["x"]);
        msg.position.y = float.Parse(pairs["y"]);
        //TODO STRING -> DANGER_CASE
        msg.situation = pairs["situation"];

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

        return msg;
    }
}

//Json Parsing Output Class
public class PositionMsg
{
    public ACTION_STATE actionState;
    public Vector2 position;
    public string situation;
}

public class StatusMsg
{
    public string serialNumber;
    public AMR_STATE state;
    public Vector2 position;
    public float speed;
}