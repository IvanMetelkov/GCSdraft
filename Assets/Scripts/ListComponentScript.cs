using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ListComponentScript : MonoBehaviour
{
    public GraphicComponent graphicComponent;
    // Start is called before the first frame update
    public void AddTempComponent()
    {
        if (graphicComponent.GetType() == typeof(Point2D))
        {
            Point2D tmp = (Point2D)graphicComponent;
            WindowManager.tempPoints.Add(tmp.point);
            WindowManager.DisableQuickMenu();
        }
    }
}
