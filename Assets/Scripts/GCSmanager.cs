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
        origin.isFixed = true;
        constraints = new List<Constraint>();
        failedConstraints = new List<Constraint>();
        points = new List<Point>();
        segments = new List<Segment>();

        CreatePoint(0, 1);
        CreatePoint(1, 0);
        //AddConstraint(new Distance(points[0], points[1], 10.0f));
     
        //AddConstraint(new Alignment(points[0], points[1]));
        //AddConstraint(new Distance(points[0], points[1], 5.0f));
        //CreatePoint(10, 10);
        /*AddConstraint(new Fixation(points[0], points[0].x, points[0].y));
        AddConstraint(new Distance(points[1], origin, 1.0f));
        AddConstraint(new Distance(points[1], points[0], 10.0f));
        AddConstraint(new Distance(points[1], points[0], 5.0f));

        AddConstraint(new Fixation(points[0], points[0].x, points[0].y));
        AddConstraint(new Fixation(points[1], points[1].x, points[1].y));
        AddConstraint(new Fixation(points[0], points[0].x, points[0].y));
        AddConstraint(new Fixation(points[1], points[1].x, points[1].y));
        AddConstraint(new Fixation(points[1], points[1].x, points[1].y));*/

        /*AddConstraint(new Distance(points[0], origin, 1.0f));
        AddConstraint(new Distance(origin, points[1], 10.0f));
        AddConstraint(new Distance(points[0], origin, 1.0f));
        AddConstraint(new Distance(points[0], points[1], -1.0f));
        AddConstraint(new Distance(points[0], points[1], 1.0f));*/

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
        for (int i = constraintedPoints.Count - 1; i >= 0; --i)
        {
            //нет ну а что
            if (constraintedPoints[i].constraintCount <= 0)
            {
                matrixNF.RemoveColumn(2 * i + 1);
                matrixNF.RemoveColumn(2 * i);
                constraintedPoints.RemoveAt(i);
            }
        }

        for (int i = 0; i < constraintedPoints.Count; ++i)
        {
            constraintedPoints[i].columnID = i;
        }
    }

    private void UpdateConstraints()
    {
        for (int i = constraints.Count - 1; i >= 0; --i)
        {
            if (constraints[i].toBeDeleted)
            {
                //съедаем матрицу
                for (int j = 0; j < constraints[i].lambdas; ++j)
                {
                    matrixNF.RemoveRow(constraints[i].startingEquation);
                    freeVector.RemoveRow(constraints[i].startingEquation);
                }
                constraints.RemoveAt(i);
            }
        }
        int startingEquation = 0;
        foreach (Constraint constraint in constraints)
        {
            constraint.startingEquation = startingEquation;
            startingEquation += constraint.lambdas;
        }
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
            matrixNF = Matrix<float>.Build.Dense(constraint.lambdas, constraint.uniquePoints.Count * 2);
            freeVector = Matrix<float>.Build.Dense(constraint.lambdas, 1);
            constraint.RegisterConstraint(constraintedPoints, 0);
            constraint.FillDerivatives(matrixNF, equationCount, freeVector);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;
        }
        else
        //если уже полностью определена,
        //нет смысла переопределять
        if (!CheckState() && TestConstraint(constraint))
        {
            constraint.RegisterConstraint(constraintedPoints, equationCount);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;
        }
        else
        {
            failedConstraints.Add(constraint);
        }

    }

    public void DeleteConstraint(Constraint constraint)
    {
        constraint.toBeDeleted = true;
        constraint.MarkPoints();
        UpdatePoints();
        //о5 съедаем матрицу
        for (int j = 0; j < constraint.lambdas; ++j)
        {
            matrixNF.RemoveRow(constraint.startingEquation);
            freeVector.RemoveRow(constraint.startingEquation);
        }
        constraints.RemoveAt(constraints.FindIndex(c => c.startingEquation == constraint.startingEquation));
        int startingEquation = 0;
        foreach (Constraint c in constraints)
        {
            c.startingEquation = startingEquation;
            startingEquation += c.lambdas;
        }
    }
    public void DeletePoint(Point point)
    {
        point.MarkConstraints();
        UpdatePoints();
        points.RemoveAt(points.FindIndex(p => p.pointID == point.pointID));
        UpdateConstraints();
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
            //только, кажется, не работает
            //гонять метод Ньютона-Рафсона для проверки как есть?
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
    public bool isFixed = false;
    public int constraintCount = 0;
    public float x;
    public float y;
    public List<Constraint> relatedConstraints;

    public Point(float x, float y, int id)
    {
        this.x = x;
        this.y = y;
        pointID = id;
        columnID = -1;
        relatedConstraints = new List<Constraint>();
    }

    public bool IsOrigin()
    {
        return pointID == -1;
    }

    public void MarkConstraints()
    {
        foreach (Constraint constraint in relatedConstraints)
        {
            constraint.toBeDeleted = true;
            constraint.MarkPoints();
        }
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
    public List<Point> pointList;
    public List<Point> uniquePoints;
    public int startingEquation;
    public bool toBeDeleted = false;
    public Constraint(int lambdas, List<Point> points)
    {
        uniquePoints = new List<Point>();
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
                uniquePoints.Add(p);
                uniqueIDs.Add(p.pointID);
                p.constraintCount++;
            }
        }
    }
    //если ограничение не переопределяет систему и не
    //противоречит другим - "регистрируем" его
    public void RegisterConstraint(List<Point> constraintedPoints, int number)
    {
        startingEquation = number;
        foreach (Point p in uniquePoints)
        {
            AddPoint(p, constraintedPoints);
        }
    }

    public void MarkPoints()
    {
        foreach (Point p in uniquePoints)
        {
            p.constraintCount--;
        }
    }

    //метод заполняет СЛАУ по уравнению
    //необязательный параметр можно бы и сделать обязательным, хз
    public abstract void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1);

    public int InactivePoints() {
        int count = 0;
        foreach (Point p in uniquePoints)
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
            p.relatedConstraints.Add(this);
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
    private float distance;
    public Distance(Point p1, Point p2, float d) : base(1, new List<Point> { p1, p2 })
    {
        distance = d;
    }
    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        //опа костыли
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = 2.0f * (pointList[0].x - pointList[1].x);
            m[equationNumber, 2 * column + 1] = 2.0f * (pointList[0].y - pointList[1].y);
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 2.0f * (pointList[1].x - pointList[0].x);
            m[equationNumber, 2 * column + 1] = 2.0f * (pointList[1].y - pointList[0].y);
        }

        b[equationNumber, 0] = -(Mathf.Pow(pointList[0].x - pointList[1].x, 2.0f) + Mathf.Pow(pointList[0].y - pointList[1].y, 2.0f) - Mathf.Pow(distance, 2.0f));
    }
}

public class Fixation : Constraint
{
    //запрашивать ли новые координаты для фиксации??
    private float x;
    private float y;
    public Fixation(Point p, float newX, float newY) : base(2, new List<Point> { p })
    {
        p.isFixed = true;
        x = newX;
        y = newY;
    }

    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
        m[equationNumber, 2 * column] = 1.0f;
        m[equationNumber + 1, 2 * column + 1] = 1.0f;
        //both might as well be zero
        b[equationNumber, 0] = -(x - pointList[0].x);
        b[equationNumber + 1, 0] = -(y - pointList[0].y);
    }
}

//есть ли разница, будут ли координаты одной из точек
//в левой части Ax=b или правой
public class Alignment : Constraint
{
    public Alignment(Point p1, Point p2) : base(2, new List<Point> { p1, p2 }) { }
    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        //о5 костыли
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = -1.0f;
            m[equationNumber + 1, 2 * column + 1] = -1.0f;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 1.0f;
            m[equationNumber + 1, 2 * column + 1] = 1.0f;
        }
        b[equationNumber, 0] = -(pointList[1].x - pointList[0].x);
        b[equationNumber + 1, 0] = -(pointList[1].y - pointList[0].y);
    }
}

public class Verticality : Constraint
{
    public Verticality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }

    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        //да как так-то а
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = -1.0f;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 1.0f;
        }
        b[equationNumber, 0] = -(pointList[1].x - pointList[0].x);
    }
}

public class Horizontality : Constraint
{
    public Horizontality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }

    public override void FillDerivatives(Matrix<float> m, int equationNumber, Matrix<float> b, int startingColumn = -1)
    {
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column + 1] = -1.0f;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column + 1] = 1.0f;
        }
        b[equationNumber, 0] = -(pointList[1].y - pointList[0].y);
    }
}