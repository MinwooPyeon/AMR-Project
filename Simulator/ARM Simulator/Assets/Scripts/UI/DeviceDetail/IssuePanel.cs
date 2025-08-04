using UnityEngine;
using UnityEngine.UI;

public class IssuePanel : MonoBehaviour
{
    public Text timeStampText;
    public Text issueText;

    public void SetData(string timeStamp, string issueTextContent)
    {
        timeStampText.text = timeStamp;
        issueText.text = issueTextContent;
    }

    public void OnClicked()
    {

    }
}
