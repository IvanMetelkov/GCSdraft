using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScalableGraphic : MonoBehaviour
{
    public Vector3 startingScale;
    public bool scaleTransform = true;
    public void ScaleTransform()
    {
        if (scaleTransform)
        {
            Vector3 scale = startingScale;
            scale *= Camera.main.fieldOfView / 60;
            scale.z = startingScale.z;
            gameObject.transform.localScale = scale;
        }

        if (gameObject.TryGetComponent(out Line2D lineComponent))
        {
            lineComponent.edgeCollider.edgeRadius = Line2D.startingEdgeRadius * Camera.main.fieldOfView / 60;
            if (!lineComponent.segment.isOrigin)
                lineComponent.lineRenderer.widthMultiplier = 0.2f * Camera.main.fieldOfView / 60;
        }
    }
}
