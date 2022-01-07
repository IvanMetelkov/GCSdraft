using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

public class WindowManager : MonoBehaviour
{
    public static GameObject toolTip;
    private static GameObject quickMenu;
    private Transform graphicsHolder;
    private Canvas canvas;
    public static float tooltipOffset = 30.0f;
    public static GameObject axisText;
    [SerializeField]
    private GameObject pointPrefab;
    [SerializeField]
    private GameObject linePrefab;
    [SerializeField]
    private GameObject fixationConstraintPrefab;
    [SerializeField]
    private GameObject alignmentPrefab;
    [SerializeField]
    private GameObject genericConstraint;
    private GCSmanager gcsManager;
    public static bool primitiveDrawMode = false;
    public static bool constraintDrawMode = false;
    public static GraphicType graphicType = GraphicType.None;
    public static ConstraintType constraintType = ConstraintType.None;
    public static List<Point> tempPoints;
    [SerializeField]
    private GameObject listPrefab;
    private static GameObject listContent;
    [SerializeField]
    private Sprite pointImage;
    [SerializeField]
    private Sprite segmentImage;
    [SerializeField]
    private Sprite[] constraintSprites;
    public static int inversedPointCount = -4;
    public static GraphicComponent componentToDelete = null;
    public static List<Segment> segmentsToDelete = new List<Segment>();
    public static List<Segment> tempSegments = new List<Segment>();

    void Awake()
    {
        gcsManager = gameObject.GetComponent<GCSmanager>();
        SetupUI();
        graphicsHolder = new GameObject("Graphics").transform;
        GCSmanager.ox.isOrigin = true;
        GCSmanager.oy.isOrigin = true;
        axisText = GameObject.Find("OriginAxis");
        DrawNewSegment(GCSmanager.ox, "X-axis", Color.red);
        DrawNewSegment(GCSmanager.oy, "Y-axis", Color.green);
        DrawNewPoint(GCSmanager.origin, "Origin-(0,0)", Color.white);
        SetupScale(GCSmanager.ox.graphic.gameObject, new Vector3(Line2D.defaultScale, Line2D.defaultScale, 1f));
        SetupScale(GCSmanager.oy.graphic.gameObject, new Vector3(Line2D.defaultScale, Line2D.defaultScale, 1f));
        SetupScale(axisText, new Vector3(0.05f, 0.05f, 1));
        tempPoints = new List<Point>();
    }

    public void SetupScale(GameObject obj, Vector3 startingScale, bool setScaleTransform = true)
    {
        ScalableGraphic scaler = obj.AddComponent<ScalableGraphic>();
        scaler.startingScale = startingScale;
        scaler.scaleTransform = setScaleTransform;
        scaler.ScaleTransform();
    }

    public void DrawNewSegment(Segment segment, string tooltipText, Color color)
    {
        GameObject instance = Instantiate(linePrefab);
        Line2D lineComponent = instance.GetComponent<Line2D>();
        lineComponent.segment = segment;
        lineComponent.color = color;
        lineComponent.tooltipName = tooltipText;
        lineComponent.UpdateValues();
        lineComponent.DrawSegment();
        segment.graphic = lineComponent;
        if (!segment.isOrigin)
        {
            instance.transform.SetParent(graphicsHolder);
        }
    }

    public void DrawNewPoint(Point point, string toolTipText, Color color)
    {
        GameObject instance = Instantiate(pointPrefab);
        Point2D pointComponent = instance.GetComponent<Point2D>();
        pointComponent.point = point;
        pointComponent.color = color;
        pointComponent.tooltipName = toolTipText;
        pointComponent.UpdateValues();
        point.graphic = pointComponent;
        SetupScale(instance, new Vector3(Point2D.defaultScale, Point2D.defaultScale, 1f));
        if (!point.IsOrigin())
        {
            instance.transform.SetParent(graphicsHolder);
        }
    }

    void SetupUI()
    {
        canvas = GameObject.Find("Canvas").GetComponent<Canvas>();
        toolTip = canvas.transform.Find("ToolTip").gameObject;
        quickMenu = canvas.transform.Find("QuickSelection").gameObject;
        listContent = quickMenu.transform.Find("ScrollList").gameObject.
            transform.Find("ScrollViewPort").gameObject.
            transform.Find("Content").gameObject;
    }

    public static void ClearTemporaryGraphic()
    {
        foreach (Point p in tempPoints)
        {
            if (p.pointID <= -4)
            {
                Destroy(p.graphic.gameObject);
            }
        }
        inversedPointCount = -4;
        tempPoints.Clear();
    }

    public static List<T> GetRaycastHit<T>()
    {
        List<T> ans = new List<T>();
        Vector3 pos = Input.mousePosition;
        pos.z = CameraController.cameraToPlaneDistance;
        pos = Camera.main.ScreenToWorldPoint(pos);
        Collider2D[] colliders = Physics2D.OverlapPointAll(pos);
        foreach (Collider2D collider in colliders)
        {
            if (collider.gameObject.TryGetComponent(out T obj))
            {
                ans.Add(obj);
            }
        }
        return ans;
    }
    void Update()
    {
        if (Input.GetMouseButtonDown(0) && quickMenu.activeSelf && !IsPointerOverUIObject())
        {
            DisableQuickMenu();
        }
        else
        if (Input.GetMouseButtonDown(0) && !IsPointerOverUIObject())
        {
            Vector3 pos = Input.mousePosition;
            pos.z = CameraController.cameraToPlaneDistance;
            pos = Camera.main.ScreenToWorldPoint(pos);
            if (primitiveDrawMode)
            {
                switch (graphicType)
                {
                    case GraphicType.Point:
                        gcsManager.CreatePoint(pos.x, pos.y);
                        string tooltipName = "Point" + (gcsManager.pointsCreated - 1).ToString();
                        DrawNewPoint(gcsManager.points[gcsManager.points.Count - 1], tooltipName, Color.magenta);
                        break;
                    case GraphicType.Segment:
                        List<Point2D> points = GetRaycastHit<Point2D>();

                        if (points.Count == 0)
                        {
                            Point tmp = new Point(pos.x, pos.y, inversedPointCount);
                            inversedPointCount--;
                            DrawNewPoint(tmp, "TempPoint" + tempPoints.Count.ToString(), Color.gray);
                            tempPoints.Add(tmp);
                            if (tempPoints.Count == 2)
                            {
                                FormNewSegment();
                            }
                        }
                        else
                        {
                            FishForPoints(points);
                        }
                        break;
                    case 0:
                        break;
                }
            }
            else
            if (constraintDrawMode)
            {
                List<Point2D> points = GetRaycastHit<Point2D>();
                switch (constraintType)
                {
                    case ConstraintType.Fixation:
                    case ConstraintType.Alignment:
                        FishForPoints(points);
                        break;
                    case 0:
                        break;
                }
            }
            else
            {
                List<GraphicComponent> points = GetRaycastHit<GraphicComponent>();
                foreach(GraphicComponent p in points)
                {
                    if (p.GetType() == typeof(Point2D))
                    {
                        Point2D tmp = (Point2D)p;
                        Debug.Log(tmp.point.pointID);
                    }
                }
            }
        }
        //костылище
        if (primitiveDrawMode && tempPoints.Count == 2 && graphicType == GraphicType.Segment)
        {
            FormNewSegment();
        }

        if (tempSegments.Count == 0 && constraintDrawMode && tempPoints.Count > 0)
        {
            FormNewPointConstraint();
        }

        if (!primitiveDrawMode && !constraintDrawMode && componentToDelete != null)
        {
            Type type = componentToDelete.GetType();
            if (type == typeof(Point2D))
            {
                Point2D tmp = (Point2D)componentToDelete;
                if (!tmp.point.IsOrigin())
                {
                    gcsManager.DeletePoint(tmp.point);
                    Debug.Log(segmentsToDelete.Count);
                    foreach (Segment s in segmentsToDelete)
                    {
                        Destroy(s.graphic.gameObject);
                    }
                    Destroy(tmp.gameObject);
                    segmentsToDelete.Clear();
                }
            }
            else
            if (type == typeof(Line2D))
            {
                Line2D tmp = (Line2D)componentToDelete;
                if (!tmp.segment.isOrigin)
                {
                    gcsManager.DeleteSegment(tmp.segment);
                    tmp.segment.RemoveSegmentReference();
                    Destroy(tmp.gameObject);
                }
            }
            else
            {
                Debug.Log("delete");
                Constraint2D tmp = (Constraint2D)componentToDelete;
                gcsManager.DeleteConstraint(tmp.constraint);
                Destroy(tmp.gameObject);
                Debug.Log(gcsManager.constraintedPoints.Count);
            }
            componentToDelete = null;
        }

        if (Input.GetKey(KeyCode.Delete) && !IsPointerOverUIObject() && !primitiveDrawMode && !constraintDrawMode)
        {
            tempPoints.Clear();
            DisableQuickMenu();
            List<GraphicComponent> graphics = GetRaycastHit<GraphicComponent>();
            if (graphics.Count > 0)
            {
                PopulateList(graphics);
                EnableQuickMenu();
            }
        }

        if (Input.GetKey(KeyCode.Escape))
        {
            primitiveDrawMode = false;
            constraintDrawMode = false;
            graphicType = 0;
            constraintType = 0;
            ClearTemporaryGraphic();
            DisableQuickMenu();
        }

    }

    private void FishForPoints(List<Point2D> points)
    {
        if (points.Count == 1)
        {
            tempPoints.Add(points[0].point);
        }
        else
        if (points.Count > 1)
        {
            PopulateList(points);
            EnableQuickMenu();
        }
    }

    private Constraint2D DrawNewConstraint(Constraint constraint, string tooltipText)
    {
        GameObject instance;
        if (constraintType == ConstraintType.Fixation)
        {
             instance = Instantiate(fixationConstraintPrefab);
        }
        else
        if (constraintType == ConstraintType.Alignment)
        {
            instance = Instantiate(alignmentPrefab);
        }
        else
        {
            instance = Instantiate(genericConstraint);
        }
        Constraint2D constraintComponent = instance.GetComponent<Constraint2D>();
        constraintComponent.constraint = constraint;
        constraintComponent.tooltipName = tooltipText;
        constraintComponent.tooltip.text = tooltipText;
        SetupScale(constraintComponent.gameObject, new Vector3(Constraint2D.defaultScale, Constraint2D.defaultScale, 1f), true);
        instance.transform.SetParent(graphicsHolder);
        return constraintComponent;
    }
    private void FormNewPointConstraint()
    {
        switch (constraintType)
        {
            case ConstraintType.Fixation:
                if (!tempPoints[0].IsOrigin())
                {
                    Fixation tmp = new Fixation(tempPoints[0]);
                    if (gcsManager.AddConstraint(tmp))
                    {
                        tempPoints[0].relatedConstraints.Add(tmp);
                        tmp.graphic = DrawNewConstraint(tmp, "Fixation:" + tempPoints[0].graphic.tooltipName);
                        gcsManager.MoveGraphics();
                    }
                }
                tempPoints.Clear();
                break;
            case ConstraintType.Alignment:
                if (tempPoints.Count == 2)
                {
                    if (tempPoints[0].pointID != tempPoints[1].pointID)
                    {
                        Alignment tmp = new Alignment(tempPoints[0], tempPoints[1]);
                        if (gcsManager.AddConstraint(tmp))
                        {
                            tempPoints[0].relatedConstraints.Add(tmp);
                            tempPoints[1].relatedConstraints.Add(tmp);
                            tmp.graphic = DrawNewConstraint(tmp, "Alignment: P" + tempPoints[0].pointID + ":P" +
                                tempPoints[1].pointID);
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempPoints.Clear();
                }
                break;
            case 0:
                tempPoints.Clear();
                break;
        }
    }

    private void FormNewSegment()
    {
        if (tempPoints[0].pointID != tempPoints[1].pointID)
        {
            foreach (Point p in tempPoints)
            {
                if (p.pointID <= -4)
                {
                    p.graphic.color = Color.magenta;
                    p.graphic.tooltipName = "Point" + gcsManager.pointsCreated.ToString();
                    p.pointID = gcsManager.pointsCreated;
                    gcsManager.points.Add(p);
                    p.graphic.UpdateValues();
                    gcsManager.pointsCreated++;
                    p.graphic.gameObject.transform.SetParent(graphicsHolder);
                }
            }
            Segment newSegment = new Segment(tempPoints[0], tempPoints[1], gcsManager.segmentsCreated);
            DrawNewSegment(newSegment, "Segment" + gcsManager.segmentsCreated.ToString(), Color.blue);
            SetupScale(newSegment.graphic.gameObject, new Vector3(1f, 1f, 1f), false);
            gcsManager.segments.Add(newSegment);
            gcsManager.segmentsCreated++;
            tempPoints[0].relatedSegments.Add(newSegment);
            tempPoints[1].relatedSegments.Add(newSegment);
            tempPoints.Clear();
            inversedPointCount = -4;
        }
        else
        {
            ClearTemporaryGraphic();
        }
    }
    void EnableQuickMenu()
    {
        quickMenu.SetActive(true);
        Vector3 screenPos = Input.mousePosition;
        screenPos.x += 150f;
        quickMenu.transform.position = screenPos;
        HideToolTip();
    }

    public static void DisableQuickMenu()
    {
        foreach (Transform child in listContent.transform)
        {
            Destroy(child.gameObject);
        }
        quickMenu.SetActive(false);
    }

    void PopulateList<T>(List<T> list) where T : GraphicComponent
    {
        for (int i = 0; i < list.Count; ++i)
        {
            GameObject obj = Instantiate(listPrefab);
            obj.transform.Find("Number").GetComponent<Text>().text = (i + 1).ToString();
            obj.transform.Find("Name").GetComponent<Text>().text = list[i].tooltipName;
            Image img = obj.transform.Find("Icon").GetComponent<Image>();
            if (list[i].GetType() == typeof(Point2D))
            {
                img.sprite = pointImage;
            }
            if (list[i].GetType() == typeof(Line2D))
            {
                img.sprite = segmentImage;
            }
            if (list[i].GetType() == typeof(Constraint2D))
            {
                Constraint2D tmp = (Constraint2D)(object)list[i];
                img.sprite = constraintSprites[tmp.constraintNumber];
            }

            ListComponentScript content = obj.GetComponent<ListComponentScript>();
            content.graphicComponent = list[i];
            obj.transform.SetParent(listContent.transform);
        }
    }

    public static void ShowToolTip(string toolTipText, Vector2 pos)
    {
        Text toolTipUIText = toolTip.transform.Find("Text").GetComponent<Text>();
        toolTip.SetActive(true);
        float padding = 4.0f;
        toolTipUIText.text = toolTipText;
        Vector2 size = new Vector2(toolTipUIText.preferredWidth + padding * 2.0f,
            toolTipUIText.preferredHeight + padding * 2.0f);
        toolTip.GetComponent<RectTransform>().transform.position = new Vector2(pos.x, pos.y - tooltipOffset);
        toolTip.transform.Find("Background").GetComponent<RectTransform>().sizeDelta = size;
        toolTip.transform.SetAsLastSibling();
    }

    public static bool IsPointerOverUIObject()
    {
        PointerEventData eventDataCurrentPosition = new PointerEventData(EventSystem.current)
        {
            position = new Vector2(Input.mousePosition.x, Input.mousePosition.y)
        };
        List<RaycastResult> results = new List<RaycastResult>();
        EventSystem.current.RaycastAll(eventDataCurrentPosition, results);
        return results.Count > 0;
    }

    public static void HideToolTip()
    {
        toolTip.SetActive(false);
    }
}

public enum GraphicType
{
    None = 0,
    Point = 1,
    Segment = 2
}

public enum ConstraintType
{
    None = 0,
    Fixation = 1,
    Alignment = 2,
    Verticality = 3,
    Horizontality = 4
}