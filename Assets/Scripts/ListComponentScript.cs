using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ListComponentScript : MonoBehaviour
{
    public GraphicComponent graphicComponent;
    public void AddTempComponent()
    {
        if ((WindowManager.primitiveDrawMode || WindowManager.constraintDrawMode) && graphicComponent.GetType() == typeof(Point2D))
        {
            Point2D tmp = (Point2D)graphicComponent;
            WindowManager.tempPoints.Add(tmp.point);
            WindowManager.DisableQuickMenu();
        }
        else
        if (WindowManager.constraintDrawMode && graphicComponent.GetType() == typeof(Line2D))
        {
            Line2D tmp = (Line2D)graphicComponent;
            WindowManager.tempSegments.Add(tmp.segment);
            WindowManager.DisableQuickMenu();
        }
        else
        if (!WindowManager.primitiveDrawMode && !WindowManager.constraintDrawMode)
        {
            WindowManager.componentToDelete = graphicComponent;
            WindowManager.DisableQuickMenu();
        }

    }
}
