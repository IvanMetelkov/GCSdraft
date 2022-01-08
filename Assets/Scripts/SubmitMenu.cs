using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SubmitMenu : MonoBehaviour
{
    [SerializeField]
    private GameObject inputField;

    public void TrySubmitValue()
    {
        bool err = false;
        double tmp = GetDoubleInput(inputField, ref err);
        if (!err)
        {
            WindowManager.SetValue(tmp);
            gameObject.SetActive(false);
        }
    }
    public double GetDoubleInput(GameObject inputField, ref bool err, double minInclusive = 0.0, double maxInclusive = double.MaxValue)
    {
        double ans = 0.0;
        if (inputField.GetComponent<InputField>().text == string.Empty)
        {   
            err = true;
        }
        else
        if (double.TryParse(inputField.GetComponent<InputField>().text, out ans))
        {
            if (ans < minInclusive || ans > maxInclusive)
            {
                err = true;
            }
        }
        else
        {
            err = true;
        }
        return ans;
    }
}
