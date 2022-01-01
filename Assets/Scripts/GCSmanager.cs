using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using System;
using MathNet.Numerics.LinearAlgebra;
public class GCSmanager : MonoBehaviour
{
    //список ограничений
    private List<Constraint> constraints;
    private List<Constraint> failedConstraints;
    //список точек
    private List<Point> points;
    //список отрезков
    private List<Segment> segments;
    //иногда 1 ограничение порождает 2 уравнения в системе
    //хз, какой костыль лучше
    public int equationCount = 0;
    //точка начала координат, характерный id = -1
    [HideInInspector]
    public static Point origin = new Point(0.0f, 0.0f, -1);
    [HideInInspector]
    public static List<Point> constraintedPoints = new List<Point>();
    [HideInInspector]
    private Matrix<float> matrixNF;
    [HideInInspector]
    private Matrix<float> freeVector;
    //костыль, чтобы возвращать уникальные id точек
    public int pointsCreated = 0;

    void Awake()
    {
        Random.InitState((int)DateTime.Now.Ticks);
        constraints = new List<Constraint>();
        failedConstraints = new List<Constraint>();
        points = new List<Point>();
        segments = new List<Segment>();
        CreatePoint(0, 3);
        //CreatePoint(1, 0);
        AddConstraint(new Distance(points[0], origin, 1.0f));
        CreatePoint(1, 0);
        AddConstraint(new Distance(origin, points[1], 10.0f));
        AddConstraint(new Distance(points[0], origin, 1.0f));
        AddConstraint(new Distance(points[0], points[1], 2.0f));

        for (int i = 0; i < equationCount; ++i)
        {
            Debug.Log(matrixNF.Row(i));
            Debug.Log(freeVector.Row(i));
        }
    }

    void Update()
    {
        
    }

    private void UpdatePoints()
    {

    }

    //функция, которая будет вызываться при создании точки
    public void CreatePoint(float x, float y)
    {
        points.Add(new Point(x, y, pointsCreated));
        //points.Add(new Point(pointsCreated));
        pointsCreated++;
    }

    //функция, вызываемая при попытке добавления
    //ограничения
    public void AddConstraint(Constraint constraint)
    {
        //если уравнений в СЛАУ вообще еще нет,
        //то точно можно добавлять
        if (equationCount == 0)
        {
            matrixNF = Matrix<float>.Build.Dense(constraint.lambdas, constraint.points * 2);
            freeVector = Matrix<float>.Build.Dense(constraint.lambdas, 1);
            constraint.RegisterConstraint(constraintedPoints);
            constraint.FillDerivatives(matrixNF, equationCount, freeVector);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;
        }
        else
        //если уже полностью определена,
        //нет смысла переопределять
        if (!CheckState() && TestConstraint(constraint))
        {
            constraint.RegisterConstraint(constraintedPoints);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;
        }
        else
        {
            failedConstraints.Add(constraint);
        }

    }

    //каждое новое ограничение надо проверять на
    //1) переопределения
    //2) совместность получаемой СЛАУ
    public bool TestConstraint(Constraint constraint)
    {
        //по идее можно просто использовать equationCount
        //int startingRank = matrixNF.Rank();
        Matrix<float> tmpA = matrixNF.Clone();
        Matrix<float> tmpB = freeVector.Clone();
        tmpA = tmpA.Stack(Matrix<float>.Build.Dense(constraint.lambdas, constraintedPoints.Count * 2));
        tmpB = tmpB.Stack(Matrix<float>.Build.Dense(constraint.lambdas, 1));
        int pointsToAdd = constraint.InactivePoints();
        if (pointsToAdd > 0)
        {
            tmpA = tmpA.Append(Matrix<float>.Build.Dense(tmpA.RowCount, pointsToAdd * 2));
        }
        constraint.FillDerivatives(tmpA, equationCount, tmpB, constraintedPoints.Count);

       /* for (int i = 0; i < tmpA.RowCount; ++i)
        {
            Debug.Log(tmpA.Row(i));
            Debug.Log(tmpB.Row(i));
        }*/

        //проверяем ЛНЗ
        if (tmpA.Rank() == equationCount + constraint.lambdas)
        {
            Matrix<float> augumented = tmpA.Append(tmpB);
            //проверяем совместность, привет Кронекеру
            if (equationCount + constraint.lambdas == augumented.Rank())
            {
                matrixNF = tmpA;
                freeVector = tmpB;
                return true;
            }
        }

        return false;
    }

    //ультимативная функция, проверяющая состояние системы
    public bool CheckState()
    {
        return points.Count * 2 == equationCount;
    }
}
public class Point
{
    //как минимум уникальный идентификатор, чтобы точки можно было
    //отличать от друг друга
    public int pointID;
    //номер столбца, в котором эта точка входит в уравнение
    public int columnID;
    //чтобы отслеживать, перетащили ли только что точку, т.е,
    //надо ли для нее искать дельты и новые координаты 
    public bool wasMoved = false;
    public float x;
    public float y;

    public Point(float x, float y, int id)
    {
        this.x = x;
        this.y = y;
        pointID = id;
        columnID = -1;
    }

    public bool IsOrigin()
    {
        return pointID == -1;
    }

    //для тестов
    public Point(int id) : this(Random.Range(0.0f, 10.0f), Random.Range(0.0f, 10.0f), id) { }
}

//абстрактный класс ограничения
abstract public class Constraint
{
    //возможно надо, чтобы знать, по скольким лямбдам ждать производные
    //а еще - это количество порождаемых уравнений
    public int lambdas;
    public int points = 0;
    public List<Point> pointList;
    public Constraint(int lambdas, List<Point> points)
    {
        this.lambdas = lambdas;
        pointList = points;
        CalculateUniquePoints();
    }

    private void CalculateUniquePoints()
    {
        List<int> uniqueIDs = new List<int>();
        foreach (Point p in pointList)
        {
            if (!p.IsOrigin() && !uniqueIDs.Exists(i => i == p.pointID))
            {
                uniqueIDs.Add(p.pointID);
                points++;
            }
        }
    }
    //если ограничение не переопределяет систему и не
    //противоречит другим - "регистрируем" его
    public void RegisterConstraint(List<Point> constraintedPoints)
    {
        foreach (Point p in pointList)
        {
            AddPoint(p, constraintedPoints);
        }
    }

    //метод заполняет СЛАУ по уравнению
    public abstract void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1);

    public int InactivePoints() {
        int count = 0;
        foreach (Point p in pointList)
        {
            if (p.columnID == -1 && !p.IsOrigin())
            {
                count++;
            }
        }
        return count;
    }

    //Добавляет неактивную только что задействованную точку в список, включенных в СЛАУ
    public void AddPoint(Point p, List<Point> constraintedPoints)
    {
        if (!p.IsOrigin() && p.columnID == -1)
        {
            p.columnID = constraintedPoints.Count;
            constraintedPoints.Add(p);
        }
    }
}

//класс отрезка
public class Segment
{
    public Point p1;
    public Point p2;
    public Segment (Point p1, Point p2)
    {
        this.p1 = p1;
        this.p2 = p2;
    }
}

//класс ограничения типа 2 - "расстояние между двумя точками"
public class Distance : Constraint
{
    public float distance;
    public Distance(Point p1, Point p2, float d) : base(1, new List<Point> { p1, p2 })
    {
        distance = d;
    }
    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        if (!pointList[0].IsOrigin())
        {
            //полезли костыли
            if (pointList[0].columnID != -1)
            {
                m[equationNumber, 2 * pointList[0].columnID] = 2 * (pointList[0].x - pointList[1].x);
                m[equationNumber, 2 * pointList[0].columnID + 1] = 2 * (pointList[0].y - pointList[1].y);
            }
            else
            {
                m[equationNumber, 2 * startingColumn] = 2 * (pointList[0].x - pointList[1].x);
                m[equationNumber, 2 * startingColumn + 1] = 2 * (pointList[0].y - pointList[1].y);
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            if (pointList[1].columnID != -1)
            {
                m[equationNumber, 2 * pointList[1].columnID] = 2 * (pointList[1].x - pointList[0].x);
                m[equationNumber, 2 * pointList[1].columnID + 1] = 2 * (pointList[1].y - pointList[0].y);
            }
            else
            {
                m[equationNumber, 2 * startingColumn] = 2 * (pointList[1].x - pointList[0].x);
                m[equationNumber, 2 * startingColumn + 1] = 2 * (pointList[1].y - pointList[0].y);
            }
        }

        b[equationNumber, 0] = Mathf.Pow(pointList[0].x - pointList[1].x, 2.0f) + Mathf.Pow(pointList[0].y - pointList[1].y, 2.0f) - Mathf.Pow(distance, 2.0f);
    }
}
