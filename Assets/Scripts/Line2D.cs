using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Line2D : GraphicComponent
{
    public Segment segment;
    public LineRenderer lineRenderer;
    private EdgeCollider2D edgeCollider;
    public override void Awake()
    {
        edgeCollider = gameObject.GetComponent<EdgeCollider2D>();
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        base.Awake();
    }
    public void UpdateValues()
    {
        lineRenderer.endColor = color;
        lineRenderer.startColor = color;
        tooltip.text = tooltipName;
    }
    public void DrawSegment()
    {
        lineRenderer.SetPositions(new Vector3[] { new Vector3((float)segment.p1.x, (float)segment.p1.y, 0f),
            new Vector3((float)segment.p2.x, (float)segment.p2.y, 0f) });
        edgeCollider.points = new Vector2[] { new Vector2((float)segment.p1.x, (float)segment.p1.y),
            new Vector2((float)segment.p2.x, (float)segment.p2.y)};
    }
}
