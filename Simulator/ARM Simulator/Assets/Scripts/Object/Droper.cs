using UnityEngine;

public class Droper : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        StateData stateData = other.gameObject.GetComponent<StateData>();
        Debug.Log(other.gameObject);
        if (stateData != null && stateData.AmrState == AMR_STATE.LOAD)
        {
            
            stateData.AmrState = AMR_STATE.DROP;
            //TODO AMR LIFTING
        }
    }
}
