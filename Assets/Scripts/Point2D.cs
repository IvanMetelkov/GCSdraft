using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point2D : GraphicComponent
{
    public Point point;
    public static float defaultScale = 0.15f;
    private SpriteRenderer spriteRenderer;
    public override void Awake()
    {
        spriteRenderer = gameObject.GetComponent<SpriteRenderer>();
        base.Awake();
    }

    public void SetPosition()
    {
        transform.position = new Vector3((float)point.x, (float)point.y);
    }
    public void UpdateValues()
    {
        tooltip.text = tooltipName;
        spriteRenderer.color = color;
        SetPosition();
    }
}
