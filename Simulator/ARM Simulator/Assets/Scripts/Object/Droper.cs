using UnityEngine;

public class Droper : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        StateData stateData = other.gameObject.GetComponent<StateData>();
        if (stateData != null && stateData.AmrState == AMR_STATE.LOAD)
        {
            stateData.AmrState = AMR_STATE.DROP;
            other.gameObject.transform.GetChild(7).gameObject.SetActive(false);
        }
    }
}
