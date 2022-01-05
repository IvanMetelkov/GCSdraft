using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class Tooltip : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    public string text;

    public void OnPointerEnter(PointerEventData eventData)
    {
        WindowManager.ShowToolTip(text, new Vector2(Input.mousePosition.x, gameObject.transform.position.y 
            - gameObject.GetComponent<RectTransform>().rect.height / 2));
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        WindowManager.HideToolTip();
    }

    public void OnMouseEnter()
    {
        if (!WindowManager.IsPointerOverUIObject())
        {
            List<GraphicComponent> overlaps = WindowManager.GetRaycastHit<GraphicComponent>();
            string finalText = text;
            if (overlaps.Count > 1)
            {
                finalText += ", ...";
            }
            WindowManager.ShowToolTip(finalText, new Vector2(Input.mousePosition.x, Input.mousePosition.y - 35f));
        }
    }

    public void OnMouseExit()
    {
        if (!WindowManager.IsPointerOverUIObject())
        {
            WindowManager.HideToolTip();
        }
    }
}
