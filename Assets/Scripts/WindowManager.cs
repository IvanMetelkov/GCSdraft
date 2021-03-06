using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

/// <summary>
/// кто захочет читать следующий далее код-
/// пеняйте на себя (осторожно, костыли)
/// </summary>
public class WindowManager : MonoBehaviour
{
    public static GameObject toolTip;
    private static GameObject quickMenu;
    private static GameObject submitMenu;
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
    private GameObject parallelPrefab;
    [SerializeField]
    private GameObject perpPrefab;
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
    [SerializeField]
    private Sprite[] constraintPrefabSprites;
    public static int inversedPointCount = -4;
    public static GraphicComponent componentToDelete = null;
    public static List<Segment> segmentsToDelete = new List<Segment>();
    public static List<Segment> tempSegments = new List<Segment>();
    private static double submittedValue;
    private static bool success = false;

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
        lineComponent.iconSprite = segmentImage;
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
        pointComponent.iconSprite = pointImage;
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
        submitMenu = canvas.transform.Find("ParameterMenu").gameObject;
        listContent = quickMenu.transform.Find("ScrollList").gameObject.
            transform.Find("ScrollViewPort").gameObject.
            transform.Find("Content").gameObject;
    }

    public static void SetValue(double value)
    {
        submittedValue = value;
        submitMenu.gameObject.SetActive(false);
        success = true;
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
        tempSegments.Clear();
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
        if (Input.GetMouseButtonDown(0) && submitMenu.activeSelf && !IsPointerOverUIObject())
        {
            submitMenu.SetActive(false);
            success = false;
            submittedValue = 0.0;
            tempPoints.Clear();
            tempSegments.Clear();
        }
        else
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
                List<Line2D> lines = GetRaycastHit<Line2D>();
                switch (constraintType)
                {
                    case ConstraintType.Fixation:
                    case ConstraintType.Alignment:
                        FishForPoints(points);
                        break;
                    case ConstraintType.Distance:
                        if (tempPoints.Count != 2 && !submitMenu.activeSelf && !success)
                        {
                            FishForPoints(points);
                        }
                        break;
                    case ConstraintType.Horizontality:
                    case ConstraintType.Verticality:
                    case ConstraintType.Parallel:
                    case ConstraintType.Perpendicular:
                    case ConstraintType.Angle:
                    case ConstraintType.EqualSegments:
                        FishForLines(lines);
                        break;
                    case ConstraintType.PointOnLine:
                    case ConstraintType.PointLineDistance:
                        if (tempPoints.Count == 0)
                        {
                            FishForPoints(points);
                        }
                        if (tempSegments.Count == 0)
                        {
                            FishForLines(lines);
                        }
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
        if (tempPoints.Count == 2 && !submitMenu.activeSelf && !success && constraintDrawMode && constraintType == ConstraintType.Distance)
        {
            EnableMenu(submitMenu);
        }
        else
        if (tempPoints.Count == 1 && !submitMenu.activeSelf && !success && constraintDrawMode && constraintType == ConstraintType.PointLineDistance
            && tempSegments.Count == 1)
        {
            EnableMenu(submitMenu);
        }
        else
         if (tempSegments.Count == 2 && !submitMenu.activeSelf && !success && constraintDrawMode && constraintType == ConstraintType.Angle)
        {
            EnableMenu(submitMenu);
        }
        else
        if (tempPoints.Count == 1 && tempSegments.Count == 1 && (constraintType == ConstraintType.PointOnLine || constraintType == ConstraintType.PointLineDistance))
        {
            FormNewSegmentConstraint();
        }
        else
        if (primitiveDrawMode && tempPoints.Count == 2 && graphicType == GraphicType.Segment)
        {
            FormNewSegment();
        }
        else
        if (tempSegments.Count == 0 && constraintDrawMode && tempPoints.Count > 0)
        {
            FormNewPointConstraint();
        }
        else
        if (tempSegments.Count > 0 && constraintDrawMode && tempPoints.Count == 0)
        {
            FormNewSegmentConstraint();
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
                if (tmp.isSecondary)
                {
                    tmp = tmp.constraint.graphic;
                }
                gcsManager.DeleteConstraint(tmp.constraint);
                Destroy(tmp.gameObject);
                Debug.Log(gcsManager.constraintedPoints.Count);
            }
            componentToDelete = null;
        }

        if (Input.GetKey(KeyCode.Delete) && !IsPointerOverUIObject() && !primitiveDrawMode && !constraintDrawMode)
        {
            tempPoints.Clear();
            tempSegments.Clear();
            DisableQuickMenu();
            List<GraphicComponent> graphics = GetRaycastHit<GraphicComponent>();
            if (graphics.Count > 0)
            {
                PopulateList(graphics);
                EnableMenu(quickMenu);
            }
        }

        if (Input.GetKey(KeyCode.Escape))
        {
            ClearTemporaryGraphic();
            DisableQuickMenu();
            primitiveDrawMode = false;
            constraintDrawMode = false;
            submitMenu.SetActive(false);
            success = false;
            submittedValue = 0.0;
            graphicType = 0;
            constraintType = 0;
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
            EnableMenu(quickMenu);
        }
    }

    private void FishForLines(List<Line2D> lines)
    {
        if (lines.Count == 1)
        {
            tempSegments.Add(lines[0].segment);
        }
        else
        if (lines.Count > 1)
        {
            PopulateList(lines);
            EnableMenu(quickMenu);
        }
    }
    private Constraint2D DrawNewConstraint(Constraint constraint, string tooltipText)
    {
        GameObject instance;
        //просто запихни в массив 4HEad
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
        if (constraintType == ConstraintType.Parallel)
        {
            instance = Instantiate(parallelPrefab);
        }
        else
        if (constraintType == ConstraintType.Perpendicular)
        {
            instance = Instantiate(perpPrefab);
        }
        else
        {
            instance = Instantiate(genericConstraint);
            instance.GetComponent<SpriteRenderer>().sprite = constraintPrefabSprites[(int)constraintType - 1];
        }
        Constraint2D constraintComponent = instance.GetComponent<Constraint2D>();
        constraintComponent.iconSprite = constraintSprites[(int)constraintType - 1];
        constraintComponent.constraint = constraint;
        constraintComponent.tooltipName = tooltipText;
        constraintComponent.tooltip.text = tooltipText;
        SetupScale(constraintComponent.gameObject, new Vector3(Constraint2D.defaultScale, Constraint2D.defaultScale, 1f), true);
        instance.transform.SetParent(graphicsHolder);
        return constraintComponent;
    }

    private void FormNewSegmentConstraint()
    {
        switch (constraintType)
        {
            case ConstraintType.Horizontality:
                if (!tempSegments[0].isOrigin)
                {
                    Horizontality tmp = new Horizontality(tempSegments[0]);
                    if (gcsManager.AddConstraint(tmp))
                    {
                        tmp.AddConstraintReference();
                        tempSegments[0].constraints.Add(tmp);
                        tmp.graphic = DrawNewConstraint(tmp, "Horizontality: S" + tempSegments[0].segmentID);
                        gcsManager.MoveGraphics();
                    }
                }
                tempSegments.Clear();
                break;
            case ConstraintType.Verticality:
                if (!tempSegments[0].isOrigin)
                {
                    Verticality tmp = new Verticality(tempSegments[0]);
                    if (gcsManager.AddConstraint(tmp))
                    {
                        tempSegments[0].constraints.Add(tmp);
                        tmp.AddConstraintReference();
                        tmp.graphic = DrawNewConstraint(tmp, "Verticality: S" + tempSegments[0].segmentID);
                        gcsManager.MoveGraphics();
                    }
                }
                tempSegments.Clear();
                break;
            case ConstraintType.Parallel:
                if (tempSegments.Count == 2)
                {
                    if ((!tempSegments[0].isOrigin || !tempSegments[1].isOrigin) && tempSegments[0].segmentID != tempSegments[1].segmentID)
                    {
                        ParallelLines tmp = new ParallelLines(tempSegments[0], tempSegments[1]);
                        if (gcsManager.AddConstraint(tmp))
                        {
                            tempSegments[0].constraints.Add(tmp);
                            tempSegments[1].constraints.Add(tmp);
                            tmp.AddConstraintReference();
                            tmp.graphic = DrawNewConstraint(tmp, "Parallel: S" + tempSegments[0].segmentID + ":S" + tempSegments[1].segmentID);
                            tmp.secondaryGraphic = DrawNewConstraint(tmp, "Parallel: S" + tempSegments[1].segmentID + ":S" + tempSegments[0].segmentID);
                            tmp.secondaryGraphic.isSecondary = true;
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempSegments.Clear();
                }
                break;
            case ConstraintType.Perpendicular:
                if (tempSegments.Count == 2)
                {
                    if ((!tempSegments[0].isOrigin || !tempSegments[1].isOrigin) && tempSegments[0].segmentID != tempSegments[1].segmentID)
                    {
                        PerpendicularLines tmp = new PerpendicularLines(tempSegments[0], tempSegments[1]);
                        if (gcsManager.AddConstraint(tmp))
                        {
                            tempSegments[0].constraints.Add(tmp);
                            tempSegments[1].constraints.Add(tmp);
                            tmp.AddConstraintReference();
                            tmp.graphic = DrawNewConstraint(tmp, "Perpend.: S" + tempSegments[0].segmentID + ":S" + tempSegments[1].segmentID);
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempSegments.Clear();
                }
                break;
            case ConstraintType.PointOnLine:
                if (tempPoints.Count == 1 && tempSegments.Count == 1)
                {
                    if (!tempSegments[0].isOrigin || !tempPoints[0].IsOrigin())
                    {
                        PointOnLine tmp = new PointOnLine(tempPoints[0], tempSegments[0]);
                        if (gcsManager.AddConstraint(tmp))
                        {
                            tempSegments[0].constraints.Add(tmp);
                            tmp.AddConstraintReference();
                            tmp.graphic = DrawNewConstraint(tmp, "Point P" + tempPoints[0].pointID + " on S" + tempSegments[0].segmentID);
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempSegments.Clear();
                    tempPoints.Clear();
                }
                break;
            case ConstraintType.Angle:
                if (success)
                {
                    if (tempSegments.Count == 2)
                    {
                        if ((!tempSegments[0].isOrigin || !tempSegments[1].isOrigin) && tempSegments[0].segmentID != tempSegments[1].segmentID)
                        {
                            Angle tmp = new Angle(tempSegments[0], tempSegments[1], submittedValue);
                            if (gcsManager.AddConstraint(tmp))
                            {
                                tempSegments[0].constraints.Add(tmp);
                                tempSegments[1].constraints.Add(tmp);
                                tmp.AddConstraintReference();
                                tmp.graphic = DrawNewConstraint(tmp, "Angle " + tmp.angleDeg + "-S:" + tempSegments[0].segmentID + ":S"
                                    + tempSegments[1].segmentID);
                                gcsManager.MoveGraphics();
                            }
                        }
                        tempSegments.Clear();
                    }
                    success = false;
                }
                break;
            case ConstraintType.EqualSegments:
                if (tempSegments.Count == 2)
                {
                    if ((!tempSegments[0].isOrigin || !tempSegments[1].isOrigin) && tempSegments[0].segmentID != tempSegments[1].segmentID)
                    {
                        EqualSegments tmp = new EqualSegments(tempSegments[0], tempSegments[1]);
                        if (gcsManager.AddConstraint(tmp))
                        {
                            tempSegments[0].constraints.Add(tmp);
                            tempSegments[1].constraints.Add(tmp);
                            tmp.AddConstraintReference();
                            tmp.graphic = DrawNewConstraint(tmp, "Equal-S:" + tempSegments[0].segmentID + ":S" + tempSegments[1].segmentID);
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempSegments.Clear();
                }
                break;
            case ConstraintType.PointLineDistance:
                if (success)
                {
                    if (tempPoints.Count == 1 && tempSegments.Count == 1)
                    {
                        if (!tempSegments[0].isOrigin || !tempPoints[0].IsOrigin())
                        {
                            PointLineDistance tmp = new PointLineDistance(tempPoints[0], tempSegments[0], submittedValue);
                            if (gcsManager.AddConstraint(tmp))
                            {
                                tempSegments[0].constraints.Add(tmp);
                                tmp.AddConstraintReference();
                                tmp.graphic = DrawNewConstraint(tmp, "Distance " + tmp.distance + " P" + tempPoints[0].pointID + "-S" + tempSegments[0].segmentID);
                                gcsManager.MoveGraphics();
                            }
                        }
                        tempSegments.Clear();
                        tempPoints.Clear();
                    }
                    success = false;
                }
                break;
            case 0:
                tempSegments.Clear();
                break;
        }
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
                        tmp.AddConstraintReference();
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
                            tmp.AddConstraintReference();
                            tmp.graphic = DrawNewConstraint(tmp, "Alignment: P" + tempPoints[0].pointID + ":P" +
                                tempPoints[1].pointID);
                            gcsManager.MoveGraphics();
                        }
                    }
                    tempPoints.Clear();
                }
                break;
            case ConstraintType.Distance:
                if (tempPoints.Count == 2)
                {
                    if (success)
                    {
                        if (tempPoints[0].pointID != tempPoints[1].pointID)
                        {
                            Distance tmp = new Distance(tempPoints[0], tempPoints[1], submittedValue);
                            if (gcsManager.AddConstraint(tmp))
                            {
                                tmp.AddConstraintReference();
                                tmp.graphic = DrawNewConstraint(tmp, "Distance: P" + tempPoints[0].pointID + ":P" +
                                    tempPoints[1].pointID);
                                gcsManager.MoveGraphics();
                            }
                        }
                        tempPoints.Clear();
                        success = false;
                    }
                }
                break;
            case 0:
                tempPoints.Clear();
                break;
        }
    }

    public static void DisableSubmitMenu()
    {
        submitMenu.SetActive(false);
        success = false;
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
    void EnableMenu(GameObject menu)
    {
        menu.SetActive(true);
        Vector3 screenPos = Input.mousePosition;
        screenPos.x += 150f;
        menu.transform.position = screenPos;
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
            img.sprite = list[i].iconSprite;
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
    None,
    Point,
    Segment
}

public enum ConstraintType
{
    None,
    Fixation,
    Alignment,
    Verticality,
    Horizontality,
    Distance,
    Parallel,
    Perpendicular,
    PointOnLine,
    Angle,
    EqualSegments,
    PointLineDistance
}