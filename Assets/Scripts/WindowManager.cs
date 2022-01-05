using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;


public class WindowManager : MonoBehaviour
{
    // Start is called before the first frame update
    public static GameObject toolTip;
    public Canvas canvas;
    public static float tooltipOffset = 30.0f;
    [SerializeField]
    private GameObject pointPrefab;
    [SerializeField]
    private GameObject linePrefab;
    private GCSmanager gcsManager;
    void Awake()
    {
        gcsManager = gameObject.GetComponent<GCSmanager>();
        SetupUI();
        Segment ox = new Segment(GCSmanager.origin, GCSmanager.OX);
        Segment oy = new Segment(GCSmanager.origin, GCSmanager.OY);
        ox.isOrigin = true;
        oy.isOrigin = true;
        DrawNewSegment(ox, "X-axis", Color.red);
        DrawNewSegment(oy, "Y-axis", Color.green);
        DrawNewPoint(GCSmanager.origin, "Origin-(0,0)", Color.white);

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
    }

    public void DrawNewPoint(Point point, string toolTipText, Color color)
    {
        GameObject instance = Instantiate(pointPrefab);
        Point2D pointComponent = instance.GetComponent<Point2D>();
        pointComponent.point = point;
        pointComponent.color = color;
        pointComponent.tooltipName = toolTipText;
        pointComponent.UpdateValues();
    }

    void SetupUI()
    {
        canvas = GameObject.Find("Canvas").GetComponent<Canvas>();
        toolTip = canvas.gameObject.transform.Find("ToolTip").gameObject;
    }
    // Update is called once per frame
    void Update()
    {
        
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
        toolTip.gameObject.SetActive(false);
    }
}
