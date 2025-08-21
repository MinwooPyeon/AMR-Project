using UnityEngine;

public class Loader : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        StateData stateData = other.gameObject.GetComponent<StateData>();
        if (stateData != null && (stateData.AmrState == AMR_STATE.DROP || stateData.AmrState == AMR_STATE.RUNNING))
        {
            stateData.AmrState = AMR_STATE.LOAD;
            other.gameObject.transform.GetChild(7).gameObject.SetActive(true);
        }
    }
}