using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

public class WindowManager : MonoBehaviour
{
    public static GameObject toolTip;
    private static GameObject quickMenu;
    private Canvas canvas;
    public static float tooltipOffset = 30.0f;
    public static GameObject axisText;
    [SerializeField]
    private GameObject pointPrefab;
    [SerializeField]
    private GameObject linePrefab;
    private GCSmanager gcsManager;
    public static bool primitiveDrawMode = false;
    public static bool constraintDrawMode = false;
    public static GraphicType graphicType = GraphicType.None;
    public static List<Point> tempPoints;
    [SerializeField]
    private GameObject listPrefab;
    private static GameObject listContent;
    [SerializeField]
    private Sprite pointImage;
    [SerializeField]
    private Sprite segmentImage;

    void Awake()
    {
        gcsManager = gameObject.GetComponent<GCSmanager>();
        SetupUI();
        GCSmanager.ox.isOrigin = true;
        GCSmanager.oy.isOrigin = true;
        axisText = GameObject.Find("OriginAxis");
        DrawNewSegment(GCSmanager.ox, "X-axis", Color.red);
        DrawNewSegment(GCSmanager.oy, "Y-axis", Color.green);
        DrawNewPoint(GCSmanager.origin, "Origin-(0,0)", Color.white);
        SetupScale(GCSmanager.ox.graphic.gameObject, new Vector3(1f, 1f, 1f));
        SetupScale(GCSmanager.oy.graphic.gameObject, new Vector3(1f, 1f, 1f));
        SetupScale(axisText, new Vector3(0.01f, 0.01f, 1));
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
        SetupScale(instance, new Vector3(0.03f, 0.03f, 1f));
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
            if (p.pointID == -4)
            {
                Destroy(p.graphic.gameObject);
            }
        }
        tempPoints.Clear();
    }

    public static List<T> GetRaycastHit<T>()
    {
        List<T> ans = new List<T>();
        Vector3 pos = Input.mousePosition;
        pos.z = 10.0f;
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
    // Update is called once per frame
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
            pos.z = 10.0f;
            pos = Camera.main.ScreenToWorldPoint(pos);
            if (primitiveDrawMode)
            {
                switch (graphicType)
                {
                    case (GraphicType.Point):
                        gcsManager.CreatePoint(pos.x, pos.y);
                        string tooltipName = "Point" + (gcsManager.points.Count - 1).ToString();
                        DrawNewPoint(gcsManager.points[gcsManager.points.Count - 1], tooltipName, Color.magenta);
                        break;
                    case (GraphicType.Segment):
                        List<Point2D> points = GetRaycastHit<Point2D>();

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
                        else
                        {
                            Point tmp = new Point(pos.x, pos.y, -4);
                            DrawNewPoint(tmp, "TempPoint" + tempPoints.Count.ToString(), Color.gray);
                            tempPoints.Add(tmp);
                            if (tempPoints.Count == 2)
                            {
                                FormNewSegment();
                            }
                        }
                        break;
                    case (0):
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

        if (Input.GetKey(KeyCode.Delete) && !IsPointerOverUIObject() && !primitiveDrawMode && !constraintDrawMode)
        {
            DisableQuickMenu();
            List<GraphicComponent> points = GetRaycastHit<GraphicComponent>();
            if (points.Count > 0)
            {
                PopulateList(points);
                EnableQuickMenu();
            }
        }

        if (Input.GetKey(KeyCode.Escape))
        {
            primitiveDrawMode = false;
            constraintDrawMode = false;
            graphicType = 0;
            ClearTemporaryGraphic();
            DisableQuickMenu();
        }

    }

    private void FormNewSegment()
    {
        foreach (Point p in tempPoints)
        {
            if (p.pointID == -4)
            {
                p.graphic.color = Color.magenta;
                p.graphic.tooltipName = "Point" + gcsManager.points.Count.ToString();
                p.pointID = gcsManager.points.Count;
                gcsManager.points.Add(p);
                p.graphic.UpdateValues();
                gcsManager.pointsCreated++;
            }
        }
        Segment newSegment = new Segment(tempPoints[0], tempPoints[1]);
        DrawNewSegment(newSegment, "Segment" + gcsManager.segments.Count.ToString(), Color.blue);
        SetupScale(newSegment.graphic.gameObject, new Vector3(1f, 1f, 1f), false);
        gcsManager.segments.Add(newSegment);
        tempPoints.Clear();
    }
    void EnableQuickMenu()
    {
        quickMenu.SetActive(true);
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