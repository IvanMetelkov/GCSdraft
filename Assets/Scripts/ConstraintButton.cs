using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstraintButton : MonoBehaviour
{
    [SerializeField]
    private ConstraintType constraintType;

    public void SetDrawingMode()
    {
        WindowManager.ClearTemporaryGraphic();
        WindowManager.DisableQuickMenu();
        WindowManager.DisableSubmitMenu();
        WindowManager.constraintType = constraintType;
        WindowManager.graphicType = 0;
        WindowManager.primitiveDrawMode = false;
        WindowManager.constraintDrawMode = true;
    }
}
