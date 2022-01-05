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
    public static double precision = 1E-8;
    //точка начала координат, характерный id = -1
    public static Point origin = new Point(0.0, 0.0, -1);
    public static Point OX = new Point(1.0, 0.0, -2);
    public static Point OY = new Point(0.0, 1.0, -3);
    [HideInInspector]
    public List<Point> constraintedPoints;
    private Matrix<double> matrixNF;
    private Matrix<double> freeVector;

    private Matrix<double> consJacobian;
    private Matrix<double> consB;
    private Matrix<double> consDeltas;
    //костыль, чтобы возвращать уникальные id точек
    //как вариант GUID
    public int pointsCreated = 0;

    void Awake()
    {
        Random.InitState((int)DateTime.Now.Ticks);
        origin.isFixed = true;
        OX.isFixed = true;
        OY.isFixed = true;
        constraints = new List<Constraint>();
        failedConstraints = new List<Constraint>();
        points = new List<Point>();
        constraintedPoints = new List<Point>();
        segments = new List<Segment>();

     
        CreatePoint(-90000.1, 1.1);
        CreatePoint(10, -777.777);
        CreatePoint(500.3034, 5);
        AddConstraint(new Alignment(points[0], points[2]));
        AddConstraint(new Horizontality(points[0], points[1]));
        AddConstraint(new Verticality(points[2], points[1]));
        //AddConstraint(new Verticality(points[0], points[1]));
       // AddConstraint(new Horizontality(points[0], points[1]));

        //AddConstraint(new Verticality(points[1], points[2]));
        //AddConstraint(new Fixation(points[0]));

        //AddConstraint(new Fixation(points[2]));
        //AddConstraint(new Alignment(points[1], points[2]));
        //AddConstraint(new Alignment(points[1], points[0]));
        //AddConstraint(new Fixation(points[1]));

        //AddConstraint(new Verticality(points[0], points[1]));

        //AddConstraint(new Distance(points[0], points[1], 10.0f));
        //AddConstraint(new Alignment(points[0], origin));
        //AddConstraint(new Distance(points[1], origin, 10.0f));
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



        //Debug.Log(matrixNF.ToString());
        //Debug.Log(freeVector.ToString());

        foreach (Point p in points)
        {
            p.LogPosition();
        }
        //consDeltas = consJacobian.Solve(consB);
        //Debug.Log(consJacobian.ToString());
        //Debug.Log(consDeltas.ToString());
        //Debug.Log(consB.ToString());
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

    public void BuildJacobian(Constraint c = null)
    {
        int startingL = 0, addP = 0, addL = 0;

        if (c != null)
        {
            addP = c.InactivePoints();
            addL = c.lambdas;
        }

        consJacobian = Matrix<double>.Build.Dense((constraintedPoints.Count + addP) * 2 + equationCount + addL,
            (constraintedPoints.Count + addP) * 2 + equationCount + addL);
        consB = Matrix<double>.Build.Dense((constraintedPoints.Count + addP) * 2 + equationCount + addL, 1);
        //consDeltas = Matrix<double>.Build.Dense((constraintedPoints.Count + addP) * 2 + equationCount + addL, 1);

        foreach (Constraint constraint in constraints)
        {
            constraint.FillJacobian(consJacobian, consDeltas, consB, startingL + (constraintedPoints.Count + addP) * 2);
            startingL += constraint.lambdas;
        }

        if (c !=null)
            c.FillJacobian(consJacobian, consDeltas, consB, startingL + (constraintedPoints.Count + addP) * 2, constraintedPoints.Count);

        consB = consB.Multiply(-1.0);
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

                if (constraints[i].GetType() == typeof(Fixation))
                {
                    Fixation tmp = (Fixation)constraints[i];
                    tmp.pointList[0].isFixed = false;
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
    public void CreatePoint(double x, double y)
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
            matrixNF = Matrix<double>.Build.Dense(constraint.lambdas, constraint.uniquePoints.Count * 2);
            freeVector = Matrix<double>.Build.Dense(constraint.lambdas, 1);
            constraint.RegisterConstraint(constraintedPoints, 0);
            constraint.FillDerivatives(matrixNF, equationCount, freeVector);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;

            //BuildJacobian();
            _ = Solve();

           /* consDeltas = consJacobian.Solve(consB);
            Debug.Log(consJacobian.ToString());
            Debug.Log(consDeltas.ToString());
            Debug.Log(consB.ToString());*/
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
            if (constraint.GetType() == typeof(Fixation))
            {
                Fixation tmp = (Fixation)constraint;
                tmp.pointList[0].isFixed = false;
            }
        }

    }

    public bool Solve(Constraint probe = null)
    {
        List<(double, double)> backup = new List<(double, double)>();
        foreach (Point p in constraintedPoints)
        {
            backup.Add((p.x, p.y));
        }
        int iterations = 0;

        int addP = 0, addL = 0;

        if (probe != null)
        {
            addP = probe.InactivePoints();
            addL = probe.lambdas;
            foreach (Point p in probe.uniquePoints)
            {
                if (p.columnID == -1 && !p.IsOrigin())
                {
                    backup.Add((p.x, p.y));
                }
            }
        }
        consDeltas = Matrix<double>.Build.Dense((constraintedPoints.Count + addP) * 2 + equationCount + addL, 1);
        while (iterations < 50 && EvaluateError(probe))
        {
            BuildJacobian(probe);
            consDeltas = consJacobian.Solve(consB);
            UpdatePointValues(probe);
            iterations++;
        }
        if (EvaluateError(probe))
        {
            RestorePointvalues(backup, probe);
        }
        return !EvaluateError(probe);
    }

    private void RestorePointvalues(List<(double, double)> backup, Constraint probe = null)
    {
        for (int i = 0; i < constraintedPoints.Count; ++i)
        {
            constraintedPoints[i].x = backup[i].Item1;
            constraintedPoints[i].y = backup[i].Item2;
        }

        if (probe != null)
        {
            int index = constraintedPoints.Count;
            foreach(Point p in probe.uniquePoints)
            {
                if (p.columnID == -1 && !p.IsOrigin())
                {
                    p.x = backup[index].Item1;
                    p.y = backup[index].Item2;
                    index++;
                }
            }
        }
    }
    private bool EvaluateError(Constraint probe = null)
    {
        double err = 0;
        foreach (Constraint constraint in constraints)
        {
            err += constraint.EstimateError(consDeltas);
        }

        if (probe != null)
        {
            err += probe.EstimateError(consDeltas);
        }
        //считать квадраты?
        return err >= precision;
    }
    private void UpdatePointValues(Constraint probe = null)
    {
        for (int i = 0; i < constraintedPoints.Count; ++i)
        {
            constraintedPoints[i].x += consDeltas[2 * i, 0];
            constraintedPoints[i].y += consDeltas[2 * i + 1, 0];
        }
        if (probe != null)
        {
            int index = constraintedPoints.Count;
            foreach (Point p in probe.uniquePoints)
            {
                if (p.columnID == -1)
                {
                    p.x += consDeltas[2 * index, 0];
                    p.y += consDeltas[2 * index + 1, 0];
                    index++;
                }
            }
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

        if (constraint.GetType() == typeof(Fixation))
        {
            Fixation tmp = (Fixation)constraint;
            tmp.pointList[0].isFixed = false;
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
        Matrix<double> tmpA = matrixNF.Clone();
        Matrix<double> tmpB = freeVector.Clone();
        tmpA = tmpA.Stack(Matrix<double>.Build.Dense(constraint.lambdas, constraintedPoints.Count * 2));
        tmpB = tmpB.Stack(Matrix<double>.Build.Dense(constraint.lambdas, 1));
        int pointsToAdd = constraint.InactivePoints();
        if (pointsToAdd > 0)
        {
            tmpA = tmpA.Append(Matrix<double>.Build.Dense(tmpA.RowCount, pointsToAdd * 2));
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
            //Matrix<double> augumented = tmpA.Append(tmpB);
            consDeltas = Matrix<double>.Build.Dense((constraintedPoints.Count + pointsToAdd) * 2 + equationCount + constraint.lambdas, 1);
            BuildJacobian(constraint);
            Debug.Log(consJacobian.Rank());
            Debug.Log(consJacobian.ToString());
            if (consJacobian.Rank() == (constraintedPoints.Count + pointsToAdd) * 2 + equationCount + constraint.lambdas)
            {
                if (Solve(constraint))
                {
                    matrixNF = tmpA;
                    freeVector = tmpB;
                    Debug.Log("win");
                    return true;
                }

            }
            //проверяем совместность, привет Кронекеру
            //только, кажется, не работает
            //гонять метод Ньютона-Рафсона для проверки как есть?
         /*   if (equationCount + constraint.lambdas == augumented.Rank())
            {
                matrixNF = tmpA;
                freeVector = tmpB;
                return true;
            }*/
        }
        Debug.Log("lose");
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
    public double x;
    public double y;
    public List<Constraint> relatedConstraints;

    public Point(double x, double y, int id)
    {
        this.x = x;
        this.y = y;
        pointID = id;
        columnID = -1;
        relatedConstraints = new List<Constraint>();
    }

    public void LogPosition()
    {
        Debug.Log((float)x + " ; " + (float)y);
    }

    public bool IsOrigin()
    {
        return pointID < 0;
    }

    public void MarkConstraints()
    {
        foreach (Constraint constraint in relatedConstraints)
        {
            constraint.toBeDeleted = true;
            constraint.MarkPoints();
        }
    }
  
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
   // public int consEquations;
    public Constraint(int lambdas, List<Point> points)
    {
        uniquePoints = new List<Point>();
        this.lambdas = lambdas;
        pointList = points;
        FindUniquePoints();
      //  consEquations = uniquePoints.Count * 2 + lambdas;
    }

    private void FindUniquePoints()
    {
        foreach (Point p in pointList)
        {
            if (!p.IsOrigin() && !uniquePoints.Exists(point => point.pointID == p.pointID))
            {
                uniquePoints.Add(p);
                p.constraintCount++;
            }
        }
    }

    public abstract void FillJacobian(Matrix<double> a, Matrix<double> x, Matrix<double> b, int startingColumnL, int startingColumnP = -1);

    public void FillDiagonalValue(Matrix<double> a, int column)
    {
        //a[2 * column, 2 * column] = a[2 * column, 2 * column] == 0.0f ? 1.0f : a[2 * column, 2 * column];
        if (a[2 * column, 2 * column] == 0.0)
        {
            a[2 * column, 2 * column] = 1.0;
        }
        
        if (a[2 * column + 1, 2 * column + 1] == 0.0)
        {
            a[2 * column + 1, 2 * column + 1] = 1.0;
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

   /* public List<int> PrepareColumns(int startingColumn)
    {
        List<int> indices = new List<int>();
        List<int> ans = new List<int>();
        foreach(Point p in uniquePoints)
        {
            if (!p.IsOrigin()) {
                if (p.columnID == -1)
                {
                    indices.Add(startingColumn);
                    startingColumn++;
                }
                else
                {
                    indices.Add(p.columnID);
                }
            }
        }

        foreach (Point p in pointList)
        {
            ans.Add(indices[uniquePoints.FindIndex(point => point.pointID == p.pointID)]);
        }
        return ans;
    }*/

    public void MarkPoints()
    {
        foreach (Point p in uniquePoints)
        {
            p.constraintCount--;
        }
    }

    //метод заполняет СЛАУ по уравнению
    //необязательный параметр можно бы и сделать обязательным, хз
    public abstract void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1);

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

    public abstract double EstimateError(Matrix<double> deltas);
}

//класс отрезка
public class Segment
{
    public Point p1;
    public Point p2;
    public bool isOrigin = false;
    public Segment (Point p1, Point p2)
    {
        this.p1 = p1;
        this.p2 = p2;
    }
}

//класс ограничения типа 2 - "расстояние между двумя точками"
public class Distance : Constraint
{
    private double distance;
    public Distance(Point p1, Point p2, double d) : base(1, new List<Point> { p1, p2 })
    {
        distance = d;
    }
    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        //опа костыли
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = 2.0 * (pointList[0].x - pointList[1].x);
            m[equationNumber, 2 * column + 1] = 2.0 * (pointList[0].y - pointList[1].y);
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 2.0 * (pointList[1].x - pointList[0].x);
            m[equationNumber, 2 * column + 1] = 2.0 * (pointList[1].y - pointList[0].y);
        }

        b[equationNumber, 0] = -(Math.Pow(pointList[0].x - pointList[1].x, 2.0) + Math.Pow(pointList[0].y - pointList[1].y, 2.0) - Math.Pow(distance, 2.0));
    }

    public override void FillJacobian(Matrix<double> a, Matrix<double> x, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        //List<int> indices = PrepareColumns(startingColumnP);
      /*  double c1 = pointList[1].x - pointList[0].x, c2 = pointList[1].y - pointList[0].y;
        if (pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;

        }*/
    }

    public override double EstimateError(Matrix<double> deltas)
    {
        return Math.Abs(Math.Pow(pointList[0].x - pointList[1].x, 2.0) + Math.Pow(pointList[0].y - pointList[1].y, 2.0) - Math.Pow(distance, 2.0));
    }
}

public class Fixation : Constraint
{
    public Fixation(Point p) : base(2, new List<Point> { p }) 
    {
        p.isFixed = true;
    }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
        m[equationNumber, 2 * column] = 1.0;
        m[equationNumber + 1, 2 * column + 1] = 1.0;

        b[equationNumber, 0] = 0.0;
        b[equationNumber + 1, 0] = 0.0;
    }

    public override void FillJacobian(Matrix<double> a, Matrix<double> x, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        //протестировать диагональную матрицу, может сработает
        int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
        FillDiagonalValue(a, column);
        a[startingColumnL, startingColumnL] = 1.0;
        a[startingColumnL + 1, startingColumnL + 1] = 1.0;
    }

    public override double EstimateError(Matrix<double> deltas)
    {
        return 0.0;
    }
}

//есть ли разница, будут ли координаты одной из точек
//в левой части Ax=b или правой
public class Alignment : Constraint
{
    public Alignment(Point p1, Point p2) : base(2, new List<Point> { p1, p2 }) { }
    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        //о5 костыли
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = -1.0;
            m[equationNumber + 1, 2 * column + 1] = -1.0;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 1.0;
            m[equationNumber + 1, 2 * column + 1] = 1.0;
        }
        b[equationNumber, 0] = -(pointList[1].x - pointList[0].x);
        b[equationNumber + 1, 0] = -(pointList[1].y - pointList[0].y);
    }

    public override void FillJacobian(Matrix<double> a, Matrix<double> deltas, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].x - pointList[0].x;
        b[startingColumnL + 1, 0] = pointList[1].y - pointList[0].y;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column);
            a[2 * column, startingColumnL] = -1.0;
            a[2 * column + 1, startingColumnL + 1] = -1.0;
            a[startingColumnL, 2 * column] = -1.0;
            a[startingColumnL + 1, 2 * column + 1] = -1.0;
            b[startingColumnL, 0] -= deltas[2 * column, 0];
            b[startingColumnL + 1, 0] -= deltas[2 * column + 1, 0];

            b[2 * column, 0] += deltas[2 * column, 0] - deltas[startingColumnL, 0];
            b[2 * column + 1, 0] += deltas[2 * column + 1, 0] - deltas[startingColumnL + 1, 0];
            if (pointList[0].columnID == -1)
            {
                startingColumnP++;
            }
        }
        if (!pointList[1].IsOrigin() && !pointList[1].isFixed)
        {
            int column = pointList[1].columnID == -1 ? startingColumnP : pointList[1].columnID;
            FillDiagonalValue(a, column);

            a[2 * column, startingColumnL] = 1.0;
            a[2 * column + 1, startingColumnL + 1] = 1.0;
            a[startingColumnL, 2 * column] = 1.0;
            a[startingColumnL + 1, 2 * column + 1] = 1.0;
            b[startingColumnL, 0] += deltas[2 * column, 0];
            b[startingColumnL + 1, 0] += deltas[2 * column + 1, 0];

            b[2 * column, 0] += deltas[2 * column, 0] + deltas[startingColumnL, 0];
            b[2 * column + 1, 0] += deltas[2 * column + 1, 0] + deltas[startingColumnL + 1, 0];
        }
    }

    public override double EstimateError(Matrix<double> deltas)
    {
        return Math.Abs(pointList[0].x - pointList[1].x) + Math.Abs(pointList[0].y - pointList[1].y);
    }
}

public class Verticality : Constraint
{
    public Verticality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        //да как так-то а
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = -1.0;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column] = 1.0;
        }
        b[equationNumber, 0] = -(pointList[1].x - pointList[0].x);
    }

    public override void FillJacobian(Matrix<double> a, Matrix<double> deltas, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].x - pointList[0].x;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column);

            b[startingColumnL, 0] -= deltas[2 * column, 0];
            b[2 * column, 0] += deltas[2 * column, 0] - deltas[startingColumnL, 0];
            a[2 * column, startingColumnL] = -1.0;
            a[startingColumnL, 2 * column] = -1.0;


            if (pointList[0].columnID == -1)
            {
                startingColumnP++;
            }
        }
        if (!pointList[1].IsOrigin() && !pointList[1].isFixed)
        {
            int column = pointList[1].columnID == -1 ? startingColumnP : pointList[1].columnID;
            FillDiagonalValue(a, column);

            b[startingColumnL, 0] += deltas[2 * column, 0];
            b[2 * column, 0] += deltas[2 * column, 0] + deltas[startingColumnL, 0];
            a[2 * column, startingColumnL] = 1.0;
            a[startingColumnL, 2 * column] = 1.0;
        }
    }

    public override double EstimateError(Matrix<double> deltas)
    {
        //проверка на p1.y != p0.y?
        //return Math.Abs(pointList[1].x - pointList[0].x);
        //return Math.Abs(pointList[1].y - pointList[0].y) < GCSmanager.precision ? GCSmanager.precision : Math.Abs(pointList[1].x - pointList[0].x);
        return Math.Abs(pointList[1].y - pointList[0].y) == 0.0 ? GCSmanager.precision : Math.Abs(pointList[1].x - pointList[0].x);

    }
}

public class Horizontality : Constraint
{
    public Horizontality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column + 1] = -1.0;
            if (pointList[0].columnID == -1)
            {
                startingColumn++;
            }
        }
        if (!pointList[1].IsOrigin())
        {
            int column = pointList[1].columnID == -1 ? startingColumn : pointList[1].columnID;
            m[equationNumber, 2 * column + 1] = 1.0;
        }
        b[equationNumber, 0] = -(pointList[1].y - pointList[0].y);
    }

    public override void FillJacobian(Matrix<double> a, Matrix<double> deltas, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].y - pointList[0].y;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column);

            b[startingColumnL, 0] -= deltas[2 * column + 1, 0];
            b[2 * column + 1, 0] += deltas[2 * column + 1, 0] - deltas[startingColumnL, 0];
            a[2 * column + 1, startingColumnL] = -1.0;
            a[startingColumnL, 2 * column + 1] = -1.0;


            if (pointList[0].columnID == -1)
            {
                startingColumnP++;
            }
        }
        if (!pointList[1].IsOrigin() && !pointList[1].isFixed)
        {
            int column = pointList[1].columnID == -1 ? startingColumnP : pointList[1].columnID;
            FillDiagonalValue(a, column);

            b[startingColumnL, 0] += deltas[2 * column + 1, 0];
            b[2 * column + 1, 0] += deltas[2 * column + 1, 0] + deltas[startingColumnL, 0];
            a[2 * column + 1, startingColumnL] = 1.0;
            a[startingColumnL, 2 * column + 1] = 1.0;
        }
    }

    public override double EstimateError(Matrix<double> deltas)
    {
        //return Math.Abs(pointList[1].y - pointList[0].y);
        //return Math.Abs(pointList[1].x - pointList[0].x) < GCSmanager.precision ? GCSmanager.precision : Math.Abs(pointList[1].y - pointList[0].y);
        return Math.Abs(pointList[1].x - pointList[0].x) == 0.0 ? GCSmanager.precision : Math.Abs(pointList[1].y - pointList[0].y);
    }
}