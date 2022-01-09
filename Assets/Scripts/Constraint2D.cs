using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Constraint2D : GraphicComponent
{
    public Constraint constraint;
    public static float defaultScale = 4.5f;
    public int constraintNumber;
    public bool isSecondary = false;

    public override void Awake()
    {
        constraintNumber = (int)WindowManager.constraintType - 1;
        base.Awake();
    }
}
