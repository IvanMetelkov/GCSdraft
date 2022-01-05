using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    Vector3 touchStart;
    bool dragFlag = false;
    float startingFOV;
    float mouseScrollMultiplier = 4.0f;
    private GCSmanager gcsManager;
    void Awake()
    {
        startingFOV = Camera.main.fieldOfView;
        gcsManager = gameObject.GetComponent<GCSmanager>();
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(2))
        {
            if (WindowManager.IsPointerOverUIObject())
            {
                dragFlag = true;
            }
            else
            {
                dragFlag = false;
                touchStart = Input.mousePosition;
                touchStart.z = 10.0f;
                touchStart = Camera.main.ScreenToWorldPoint(touchStart);
            }

        }
        if (Input.GetMouseButton(2))
        {
            if (!WindowManager.IsPointerOverUIObject() && !dragFlag)
            {
                Vector3 tmp = Input.mousePosition;
                tmp.z = 10.0f;
                Vector3 direction = touchStart - Camera.main.ScreenToWorldPoint(tmp);
                Camera.main.transform.position += direction;
            }
        }

        if (Input.GetKey(KeyCode.T))
        {
            Camera.main.fieldOfView = startingFOV;
            Camera.main.transform.position = new Vector3(0f, 0f, -10f);
            UpdateScale();
        }

        if (Input.GetAxis("Mouse ScrollWheel") != 0 && !WindowManager.IsPointerOverUIObject())
        {
            if (Camera.main.fieldOfView + -mouseScrollMultiplier * Input.mouseScrollDelta.y > 1)
            {
                Camera.main.fieldOfView += -mouseScrollMultiplier * Input.mouseScrollDelta.y;
                UpdateScale();
            }
        }

    }

    public void UpdateScale()
    {
        GCSmanager.ox.graphic.gameObject.GetComponent<ScalableGraphic>().ScaleTransform();
        GCSmanager.oy.graphic.gameObject.GetComponent<ScalableGraphic>().ScaleTransform();
        GCSmanager.origin.graphic.gameObject.GetComponent<ScalableGraphic>().ScaleTransform();
        WindowManager.axisText.GetComponent<ScalableGraphic>().ScaleTransform();
        foreach (Point p in gcsManager.points)
        {
            p.graphic.gameObject.GetComponent<ScalableGraphic>().ScaleTransform();
        }

        foreach(Segment s in gcsManager.segments)
        {
            s.graphic.gameObject.GetComponent<ScalableGraphic>().ScaleTransform();
        }
    }
}
