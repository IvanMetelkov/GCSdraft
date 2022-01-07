using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstraintButton : MonoBehaviour
{
    [SerializeField]
    private ConstraintType constraintType;

    public void SetDrawingMode()
    {
        WindowManager.constraintType = constraintType;
        WindowManager.primitiveDrawMode = false;
        WindowManager.constraintDrawMode = true;
        WindowManager.ClearTemporaryGraphic();
        WindowManager.DisableQuickMenu();
    }
}
