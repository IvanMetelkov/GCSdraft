using UnityEngine;

public class PrimitiveButton : MonoBehaviour
{
    [SerializeField]
    private GraphicType graphicType;

    public void SetDrawingMode()
    {
        WindowManager.ClearTemporaryGraphic();
        WindowManager.DisableQuickMenu();
        WindowManager.DisableSubmitMenu();
        WindowManager.graphicType = graphicType;
        WindowManager.constraintType = 0;
        WindowManager.primitiveDrawMode = true;
        WindowManager.constraintDrawMode = false;
    }
}
