using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class GraphicComponent : MonoBehaviour
{
    public Color color;
    public string tooltipName;
    public Tooltip tooltip;

    public virtual void Awake()
    {
        tooltip = gameObject.GetComponent<Tooltip>();
    }
}
