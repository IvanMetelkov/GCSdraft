using UnityEngine;

public class PrimitiveButton : MonoBehaviour
{
    [SerializeField]
    private GraphicType graphicType;

    public void SetDrawingMode()
    {
        WindowManager.graphicType = graphicType;
        WindowManager.primitiveDrawMode = true;
        WindowManager.constraintDrawMode = false;
        WindowManager.ClearTemporaryGraphic();
        WindowManager.DisableQuickMenu();
    }
}
