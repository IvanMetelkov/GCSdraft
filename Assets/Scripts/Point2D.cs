using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point2D : GraphicComponent
{
    public Point point;
    private SpriteRenderer spriteRenderer;
    // Start is called before the first frame update
    public override void Awake()
    {
        spriteRenderer = gameObject.GetComponent<SpriteRenderer>();
        base.Awake();
    }

    // Update is called once per frame
    public void UpdateValues()
    {
        tooltip.text = tooltipName;
        spriteRenderer.color = color;
        transform.position = new Vector3((float)point.x, (float)point.y);
    }
}
