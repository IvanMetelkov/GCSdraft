using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using System;
using MathNet.Numerics.LinearAlgebra;
public class GCSmanager : MonoBehaviour
{
    //список ограничений
    public List<Constraint> constraints;
    private List<Constraint> failedConstraints;
    public static float constraintOffset = 20f;
    //список точек
    [HideInInspector]
    public List<Point> points;
    //список отрезков
    public List<Segment> segments;
    //иногда 1 ограничение порождает 2 уравнения в системе
    //хз, какой костыль лучше
    public int equationCount = 0;
    public static double precision = 1e-5;
    //точка начала координат, характерный id = -1
    public static Point origin = new Point(0.0, 0.0, -1);
    public static Point OX = new Point(1.0, 0.0, -2);
    public static Point OY = new Point(0.0, 1.0, -3);
    public static Segment ox = new Segment(origin, OX, -1);
    public static Segment oy = new Segment(origin, OY, -2);
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
    public int segmentsCreated = 0;

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
    }

    private void UpdatePoints()
    {
        for (int i = constraintedPoints.Count - 1; i >= 0; --i)
        {
            //нет ну а что
            if (constraintedPoints[i].constraintCount <= 0)
            {
                matrixNF = matrixNF.RemoveColumn(2 * i + 1);
                matrixNF = matrixNF.RemoveColumn(2 * i);
                constraintedPoints[i].columnID = -1;
                Debug.Log("Unconstrained");
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
            constraint.FillJacobian(consJacobian, consB, startingL + (constraintedPoints.Count + addP) * 2);
            startingL += constraint.lambdas;
        }

        if (c !=null)
            c.FillJacobian(consJacobian, consB, startingL + (constraintedPoints.Count + addP) * 2, constraintedPoints.Count);

       // Debug.Log(consJacobian);
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
                    matrixNF = matrixNF.RemoveRow(constraints[i].startingEquation);
                    freeVector = freeVector.RemoveRow(constraints[i].startingEquation);
                }

                if (constraints[i].GetType() == typeof(Fixation))
                {
                    Fixation tmp = (Fixation)constraints[i];
                    tmp.pointList[0].isFixed = false;
                }
                constraints[i].ReleaseSegments();

                equationCount -= constraints[i].lambdas;
                foreach(Point p in constraints[i].uniquePoints)
                {
                    p.relatedConstraints.RemoveAt(p.relatedConstraints.FindIndex(c => c.startingEquation == constraints[i].startingEquation));
                }

                Destroy(constraints[i].graphic.gameObject);

                //test
                constraints[i].pointList.Clear();
                constraints[i].uniquePoints.Clear();
                constraints[i].constraintedSegments.Clear();

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
        pointsCreated++;
    }

    //функция, вызываемая при попытке добавления
    //ограничения
    public bool AddConstraint(Constraint constraint)
    {
        foreach (Constraint c in constraints)
        {
            c.columns.Clear();
        }
        //constraint.columns.Clear();

        foreach (Constraint c in constraints)
        {
            c.PrepareColumns();
        }
        constraint.PrepareColumns(constraintedPoints.Count);


        //если уравнений в СЛАУ вообще еще нет,
        //то точно можно добавлять
        if (equationCount == 0)
        {
            if (Solve(constraint))
            {
                matrixNF = Matrix<double>.Build.Dense(constraint.lambdas, constraint.uniquePoints.Count * 2);
                freeVector = Matrix<double>.Build.Dense(constraint.lambdas, 1);
                constraint.RegisterConstraint(constraintedPoints, 0);
                constraint.FillDerivatives(matrixNF, equationCount, freeVector);
                constraints.Add(constraint);
                equationCount += constraint.lambdas;
                return true;
            }
            else
            {
                return false;
            }
            //вырождение отрезка все-таки надо ловить
            //_ = Solve();

            /* consDeltas = consJacobian.Solve(consB);
             Debug.Log(consJacobian.ToString());
             Debug.Log(consDeltas.ToString());
             Debug.Log(consB.ToString());*/
            //return true;
        }
        else
        //если уже полностью определена,
        //нет смысла переопределять
        if (!CheckState() && TestConstraint(constraint))
        {
            constraint.RegisterConstraint(constraintedPoints, equationCount);
            constraints.Add(constraint);
            equationCount += constraint.lambdas;
            return true;
        }
        else
        {
            failedConstraints.Add(constraint);
            if (constraint.GetType() == typeof(Fixation))
            {
                Fixation tmp = (Fixation)constraint;
                tmp.pointList[0].isFixed = tmp.previousState;
            }
            Debug.Log("lose");
            return false;
        }
    }

    public bool Solve(Constraint probe = null)
    {
        int iterations = 0;

        int addP = 0, addL = 0;

        if (probe != null)
        {
            addP = probe.InactivePoints();
            addL = probe.lambdas;
            foreach (Point p in probe.uniquePoints)
            {
                if (p.columnID == -1 && !p.isFixed)
                {
                    p.currentDeltas = (Random.Range(-0.3f, 0.3f), Random.Range(-0.3f, 0.3f));
                  //  Debug.Log(p.currentDeltas);
                }
            }
        }
        consDeltas = Matrix<double>.Build.Dense((constraintedPoints.Count + addP) * 2 + equationCount + addL, 1);

        //TEST TEST TEST
        for(int i = 0; i < constraintedPoints.Count; ++i)
        {
            if (!constraintedPoints[i].isFixed)
            {
                constraintedPoints[i].currentDeltas = (Random.Range(-0.3f, 0.3f), Random.Range(-0.3f, 0.3f));
             //   Debug.Log(constraintedPoints[i].currentDeltas);
            }
        }

        while (iterations < 50 && EvaluateError(probe))
        {
            BuildJacobian(probe);
            consDeltas = consJacobian.Solve(consB);
            UpdatePointValues(probe);

            foreach (Constraint c in constraints)
            {
                c.CalculateLambdas(consDeltas, (constraintedPoints.Count + addP) * 2 + c.startingEquation);
            }

            if (probe != null)
            {
                probe.CalculateLambdas(consDeltas, (constraintedPoints.Count + addP) * 2 + equationCount);
            }

            iterations++;
        }

        bool fail = EvaluateError(probe);

        if (!fail)
        {
            for (int i = 0; i < constraintedPoints.Count; ++i)
            {
                constraintedPoints[i].x += constraintedPoints[i].currentDeltas.Item1;
                constraintedPoints[i].y += constraintedPoints[i].currentDeltas.Item2;
            }
            if (probe != null)
            {
                int index = constraintedPoints.Count;
                foreach (Point p in probe.uniquePoints)
                {
                    if (p.columnID == -1)
                    {
                        p.x += p.currentDeltas.Item1;
                        p.y += p.currentDeltas.Item2;
                        index++;
                    }
                }
            }
        }

        foreach (Constraint c in constraints)
        {
            c.ResetLamdas();
        }

        foreach (Point p in constraintedPoints)
        {
            p.currentDeltas = (0.0, 0.0);
        }

        if (probe != null)
        {
            probe.ResetLamdas();
            foreach (Point p in probe.uniquePoints)
            {
                p.currentDeltas = (0.0, 0.0);
            }
        }

        return !fail;
    }
    public void MoveGraphics()
    {
        foreach (Constraint constraint in constraints)
        {
            constraint.DrawConstraint();
        }

        foreach(Point p in constraintedPoints)
        {
            p.graphic.SetPosition();
        }

        foreach(Segment s in segments)
        {
            s.graphic.DrawSegment();
        }
    }
    private bool EvaluateError(Constraint probe = null)
    {
        double err = 0;
        foreach (Constraint constraint in constraints)
        {
            err += constraint.EstimateError();
        }

        if (probe != null)
        {
            err += probe.EstimateError();
        }
        //считать квадраты?
        return err >= precision;
    }
    private void UpdatePointValues(Constraint probe = null)
    {
        for (int i = 0; i < constraintedPoints.Count; ++i)
        {
            constraintedPoints[i].currentDeltas.Item1 += consDeltas[2 * i, 0];
            constraintedPoints[i].currentDeltas.Item2 += consDeltas[2 * i + 1, 0];

            constraintedPoints[i].diagonalValueFilled = false;
        }
        if (probe != null)
        {
            int index = constraintedPoints.Count;
            foreach (Point p in probe.uniquePoints)
            {
                if (p.columnID == -1)
                {
                    p.currentDeltas.Item1 += consDeltas[2 * index, 0];
                    p.currentDeltas.Item2 += consDeltas[2 * index + 1, 0];

                    p.diagonalValueFilled = false;
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
            matrixNF = matrixNF.RemoveRow(constraint.startingEquation);
            freeVector = freeVector.RemoveRow(constraint.startingEquation);
        }

        if (constraint.GetType() == typeof(Fixation))
        {
            Fixation tmp = (Fixation)constraint;
            tmp.pointList[0].isFixed = false;
        }
        constraint.ReleaseSegments();
        foreach (Point p in constraint.uniquePoints)
        {
            p.relatedConstraints.RemoveAt(p.relatedConstraints.FindIndex(c => c.startingEquation == constraint.startingEquation));
        }
        equationCount -= constraint.lambdas;

        constraints.RemoveAt(constraints.FindIndex(c => c.startingEquation == constraint.startingEquation));

        //test
        constraint.pointList.Clear();
        constraint.uniquePoints.Clear();
        constraint.constraintedSegments.Clear();

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
        if (point.relatedSegments.Count > 0)
        {
            WindowManager.segmentsToDelete = point.relatedSegments;
            foreach (Segment s in point.relatedSegments)
            {
                s.RemoveSegmentReference(point);
                segments.RemoveAt(segments.FindIndex(seg => seg.segmentID == s.segmentID));
            }
        }
        UpdatePoints();
        points.RemoveAt(points.FindIndex(p => p.pointID == point.pointID));
        UpdateConstraints();
    }

    public void DeleteSegment(Segment segment)
    {
        foreach (Constraint constraint in segment.constraints)
        {
            constraint.toBeDeleted = true;
            constraint.MarkPoints();

            //не уверен, что нужно прям в цикле
            //but better be safe than sorry
            //UpdatePoints();
            //UpdateConstraints();
        }
        UpdatePoints();
        segments.RemoveAt(segments.FindIndex(s => s.segmentID == segment.segmentID));
        UpdateConstraints();
    }

    //каждое новое ограничение надо проверять на
    //1) переопределения
    //2) совместность получаемой СЛАУ
    public bool TestConstraint(Constraint constraint)
    {
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

        //проверяем ЛНЗ
        if (tmpA.Rank() == equationCount + constraint.lambdas)
        {
            //Matrix<double> augumented = tmpA.Append(tmpB);
            consDeltas = Matrix<double>.Build.Dense((constraintedPoints.Count + pointsToAdd) * 2 + equationCount + constraint.lambdas, 1);
            // Debug.Log(constraintedPoints.Count + " " + constraints.Count);
            BuildJacobian(constraint);
            foreach(Point p in constraintedPoints)
            {
                p.diagonalValueFilled = false;
            }

            foreach(Point p in constraint.pointList)
            {
                p.diagonalValueFilled = false;
            }
           // Debug.Log(consJacobian);
           // Debug.Log(consJacobian.Rank());
           // Debug.Log(consJacobian.ToString());
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
    public Point2D graphic;
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
    public List<Segment> relatedSegments = new List<Segment>();

    public bool diagonalValueFilled = false;
    public (double, double) currentDeltas;

    public Point(double x, double y, int id)
    {
        this.x = x;
        this.y = y;
        pointID = id;
        columnID = -1;
        relatedConstraints = new List<Constraint>();
        currentDeltas = (0.0, 0.0);
    }

    public void LogPosition()
    {
        Debug.Log(x + " ; " + y);
    }

    public bool IsOrigin()
    {
        return pointID < 0;
    }

    public void MarkConstraints()
    {
        foreach (Constraint constraint in relatedConstraints)
        {
           // if (!constraint.toBeDeleted)
         //   {
                constraint.toBeDeleted = true;
                constraint.MarkPoints();
          //  }
        }
    }
  
}

//абстрактный класс ограничения
abstract public class Constraint
{
    public int lambdas;
    public List<Point> pointList;
    public List<Point> uniquePoints;
    public int startingEquation;
    public bool toBeDeleted = false;
    public Constraint2D graphic;
    public List<Segment> constraintedSegments = new List<Segment>();
    public List<double> lambdaList = new List<double>();

    public List<int> columns = new List<int>();
    public Constraint(int lambdas, List<Point> points)
    {
        uniquePoints = new List<Point>();
        this.lambdas = lambdas;
        pointList = points;
        FindUniquePoints();
        for (int i = 0; i < lambdas; ++i)
        {
            lambdaList.Add(0.0);
        }
    }


    public void CalculateLambdas(Matrix<double> deltas, int startingIndex)
    {
        //int startingIndex = firstEquation + startingEquation;
        for (int i = 0; i < lambdas; ++i)
        {
            lambdaList[i] += deltas[startingIndex, 0];
            startingIndex++;
        }
    }

    public void ResetLamdas()
    {
        for (int i = 0; i < lambdaList.Count; ++i)
        {
            lambdaList[i] = 0.0;
        }
    }
    public void ReleaseSegments()
    {
        foreach (Segment s in constraintedSegments)
        {
            s.constraints.RemoveAt(s.constraints.FindIndex(c => c.startingEquation == startingEquation));
        }
    }

    public void AddConstraintReference()
    {
        foreach (Point p in uniquePoints)
        {
            p.relatedConstraints.Add(this);
        }
    }

    public abstract void DrawConstraint();

    private void FindUniquePoints()
    {
        foreach (Point p in pointList)
        {
            if (!p.IsOrigin() && !uniquePoints.Exists(point => point.pointID == p.pointID))
            {
                uniquePoints.Add(p);
            }
        }
    }

    public abstract void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1);

    public void FillDiagonalValue(Matrix<double> a, int column, Point point)
    {
        //if (!point.diagonalValueFilled)
        //{
            a[2 * column, 2 * column] += 1.0;
            a[2 * column + 1, 2 * column + 1] += 1.0;
            point.diagonalValueFilled = true;
        //}
    }
    //если ограничение не переопределяет систему и не
    //противоречит другим - "регистрируем" его
    public void RegisterConstraint(List<Point> constraintedPoints, int number)
    {
        startingEquation = number;
        foreach (Point p in uniquePoints)
        {
            AddPoint(p, constraintedPoints);
            p.constraintCount++;
        }
    }

    public void PrepareColumns(int startingColumn = -1)
    {
        List<int> indices = new List<int>();
        foreach(Point p in uniquePoints)
        {
           // if (!p.IsOrigin()) {
                if (p.columnID == -1)
                {
                    indices.Add(startingColumn);
                    startingColumn++;
                }
                else
                {
                    indices.Add(p.columnID);
                }
       //     }
        }

        foreach (Point p in pointList)
        {
            if (!p.IsOrigin())
            {
                columns.Add(indices[uniquePoints.FindIndex(point => point.pointID == p.pointID)]);
            }
            else
            {
                columns.Add(-1);
            }
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
            //p.relatedConstraints.Add(this);
        }
    }

    public abstract double EstimateError();
}

//класс отрезка
public class Segment
{
    public Point p1;
    public Point p2;
    public Line2D graphic;
    public bool isOrigin = false;
    public int segmentID;
    public List<Constraint> constraints = new List<Constraint>();
    public Segment (Point p1, Point p2, int id)
    {
        this.p1 = p1;
        this.p2 = p2;
        segmentID = id;
    }

    public void RemoveSegmentReference(Point p = null)
    {
        if (p == null)
        {
            p1.relatedSegments.RemoveAt(p1.relatedSegments.FindIndex(s => s.segmentID == segmentID));
            p2.relatedSegments.RemoveAt(p2.relatedSegments.FindIndex(s => s.segmentID == segmentID));
        }
        else
        {
            Point otherEnd = p1.pointID == p.pointID ? p2 : p1;
            otherEnd.relatedSegments.RemoveAt(otherEnd.relatedSegments.FindIndex(s => s.segmentID == segmentID));
        }
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

    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        double a1 = pointList[0].x - pointList[1].x + pointList[0].currentDeltas.Item1 - pointList[1].currentDeltas.Item1;
        double a2 = pointList[0].y - pointList[1].y + pointList[0].currentDeltas.Item2 - pointList[1].currentDeltas.Item2;
        b[startingColumnL, 0] = Math.Pow(a1, 2.0) + Math.Pow(a2, 2.0) - Math.Pow(distance, 2.0);

        FillRow(new List<int> { 0, 1 }, a, b, startingColumnL, a1, a2);
        FillRow(new List<int> { 1, 0 }, a, b, startingColumnL, -a1, -a2);
    }

    public void FillRow(List<int> permutation, Matrix<double> a, Matrix<double> b, int startingColumnL,
        double c1, double c2)
    {

        if (columns[permutation[0]] != -1 && !pointList[permutation[0]].isFixed)
        {
            FillDiagonalValue(a, columns[permutation[0]], pointList[permutation[0]]);

            a[2 * columns[permutation[0]], 2 * columns[permutation[0]]] += 2 * lambdaList[0];
            a[2 * columns[permutation[0]] + 1, 2 * columns[permutation[0]] + 1] += 2 * lambdaList[0];

            a[2 * columns[permutation[0]], startingColumnL] = 2 * c1;
            a[2 * columns[permutation[0]] + 1, startingColumnL] = 2 * c2;

            b[2 * columns[permutation[0]], 0] += pointList[permutation[0]].currentDeltas.Item1 + 2 * c1 * lambdaList[0];
            b[2 * columns[permutation[0]] + 1, 0] += pointList[permutation[0]].currentDeltas.Item2 + 2 * c2 * lambdaList[0];

            a[startingColumnL, 2 * columns[permutation[0]]] = 2 * c1;
            a[startingColumnL, 2 * columns[permutation[0]] + 1] = 2 * c2;

            if (columns[permutation[1]] != -1)
            {
                a[2 * columns[permutation[0]], 2 * columns[permutation[1]]] -= 2 * lambdaList[0];
                a[2 * columns[permutation[0]] + 1, 2 * columns[permutation[1]] + 1] -= 2 * lambdaList[0];
            }
        }
    }
    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)((pointList[0].x + pointList[1].x) / 2.0),
            (float)((pointList[0].y + pointList[1].y) / 2.0), 0f);
    }

    public override double EstimateError()
    {
        return Math.Abs(Math.Pow(pointList[0].x - pointList[1].x + pointList[0].currentDeltas.Item1 - pointList[1].currentDeltas.Item1, 2.0) 
            + Math.Pow(pointList[0].y - pointList[1].y + pointList[0].currentDeltas.Item2 - pointList[1].currentDeltas.Item2, 2.0) 
            - Math.Pow(distance, 2.0));
    }
}

public class Fixation : Constraint
{
    public bool previousState;
    public Fixation(Point p) : base(2, new List<Point> { p }) 
    {
        previousState = p.isFixed;
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

    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        //протестировать диагональную матрицу, может сработает
        int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
        FillDiagonalValue(a, column, pointList[0]);
        a[startingColumnL, startingColumnL] = 1.0;
        a[startingColumnL + 1, startingColumnL + 1] = 1.0;
    }

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)pointList[0].x, (float)pointList[0].y, 0f);
    }

    public override double EstimateError()
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

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)pointList[0].x, (float)pointList[0].y, 0f);
    }
    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1;
        b[startingColumnL + 1, 0] = pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column, pointList[0]);
            a[2 * column, startingColumnL] = -1.0;
            a[2 * column + 1, startingColumnL + 1] = -1.0;
            a[startingColumnL, 2 * column] = -1.0;
            a[startingColumnL + 1, 2 * column + 1] = -1.0;

            b[2 * column, 0] += pointList[0].currentDeltas.Item1 - lambdaList[0];
            b[2 * column + 1, 0] += pointList[0].currentDeltas.Item2 - lambdaList[1];
            if (pointList[0].columnID == -1)
            {
                startingColumnP++;
            }
        }
        if (!pointList[1].IsOrigin() && !pointList[1].isFixed)
        {
            int column = pointList[1].columnID == -1 ? startingColumnP : pointList[1].columnID;
            FillDiagonalValue(a, column, pointList[1]);

            a[2 * column, startingColumnL] = 1.0;
            a[2 * column + 1, startingColumnL + 1] = 1.0;
            a[startingColumnL, 2 * column] = 1.0;
            a[startingColumnL + 1, 2 * column + 1] = 1.0;

            b[2 * column, 0] += pointList[1].currentDeltas.Item1 + lambdaList[0];
            b[2 * column + 1, 0] += pointList[1].currentDeltas.Item2 + lambdaList[1];
        }
    }

    public override double EstimateError()
    {
        return Math.Abs(pointList[0].x - pointList[1].x - pointList[1].currentDeltas.Item1 + pointList[0].currentDeltas.Item1) 
            + Math.Abs(pointList[0].y - pointList[1].y - pointList[1].currentDeltas.Item2 + pointList[0].currentDeltas.Item2);
    }
}

public class Verticality : Constraint
{
    public Verticality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }

    public Verticality(Segment segment) : this(segment.p1, segment.p2) 
    {
        constraintedSegments.Add(segment);
    }
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

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)((pointList[0].x + pointList[1].x) / 2.0), 
            (float)((pointList[0].y + pointList[1].y) / 2.0), 0f);
    }
    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column, pointList[0]);

            b[2 * column, 0] += pointList[0].currentDeltas.Item1 - lambdaList[0];
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
            FillDiagonalValue(a, column, pointList[1]);

            b[2 * column, 0] += pointList[1].currentDeltas.Item1 + lambdaList[0];
            a[2 * column, startingColumnL] = 1.0;
            a[startingColumnL, 2 * column] = 1.0;
        }
    }

    public override double EstimateError()
    {
        //проверка на p1.y != p0.y?
        return Math.Abs(pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1);
        //return Math.Abs(pointList[1].y - pointList[0].y) < GCSmanager.precision ? GCSmanager.precision : Math.Abs(pointList[1].x - pointList[0].x);
        //return Math.Abs(pointList[1].y - pointList[0].y) == 0.0 ? GCSmanager.precision : Math.Abs(pointList[1].x - pointList[0].x);

    }
}

public class Horizontality : Constraint
{
    public Horizontality(Point p1, Point p2) : base(1, new List<Point> { p1, p2 }) { }
    public Horizontality(Segment segment) : this(segment.p1, segment.p2) 
    {
        constraintedSegments.Add(segment);
    }
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

    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        b[startingColumnL, 0] = pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2;
        if (!pointList[0].IsOrigin() && !pointList[0].isFixed)
        {
            int column = pointList[0].columnID == -1 ? startingColumnP : pointList[0].columnID;
            FillDiagonalValue(a, column, pointList[0]);

            b[2 * column + 1, 0] += pointList[0].currentDeltas.Item2 - lambdaList[0];
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
            FillDiagonalValue(a, column, pointList[1]);

            b[2 * column + 1, 0] += pointList[1].currentDeltas.Item2 + lambdaList[0];
            a[2 * column + 1, startingColumnL] = 1.0;
            a[startingColumnL, 2 * column + 1] = 1.0;
        }
    }

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)((pointList[0].x + pointList[1].x) / 2.0),
            (float)((pointList[0].y + pointList[1].y) / 2.0), 0f);
    }
    public override double EstimateError()
    {
        return Math.Abs(pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2);
        //return Math.Abs(pointList[1].x - pointList[0].x) < GCSmanager.precision ? GCSmanager.precision : Math.Abs(pointList[1].y - pointList[0].y);
        //return Math.Abs(pointList[1].x - pointList[0].x) == 0.0 ? GCSmanager.precision : Math.Abs(pointList[1].y - pointList[0].y);
    }
}

public class PointOnLine : Constraint
{
    public PointOnLine(Point p, Segment segment) : base (1, new List<Point> { p, segment.p1, segment.p2 })
    {
        constraintedSegments.Add(segment);
    }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        if (!pointList[0].IsOrigin())
        {
            int column = pointList[0].columnID == -1 ? startingColumn : pointList[0].columnID;
            m[equationNumber, 2 * column] = -1.0;
        }
    }

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)pointList[0].x, (float)pointList[0].y, 0f);
    }
    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
      
    }

    public override double EstimateError()
    {
        return Math.Abs(pointList[1].y - pointList[0].y) == 0.0 ? GCSmanager.precision : Math.Abs(pointList[1].x - pointList[0].x);
    }
}

public class ParallelLines : Constraint
{
    public ParallelLines(Segment s1, Segment s2) : base(1, new List<Point> { s1.p1, s1.p2, s2.p1, s2.p2 })
    {
        constraintedSegments.Add(s1);
        constraintedSegments.Add(s2);
    }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        double a1 = pointList[3].y - pointList[2].y, a2 = pointList[1].y - pointList[0].y, 
            a3 = pointList[3].x - pointList[2].x, a4 = pointList[1].x - pointList[0].x;

        if (!pointList[0].IsOrigin())
        {
            m[equationNumber, 2 * columns[0]] += -a1;
            m[equationNumber, 2 * columns[0] + 1] += a3;
        }

        if (!pointList[1].IsOrigin())
        {
            m[equationNumber, 2 * columns[1]] += a1;
            m[equationNumber, 2 * columns[1] + 1] += -a3;
        }

        if (!pointList[2].IsOrigin())
        {
            m[equationNumber, 2 * columns[2]] += a2;
            m[equationNumber, 2 * columns[2] + 1] += -a4;
        }

        if (!pointList[3].IsOrigin())
        {
            m[equationNumber, 2 * columns[3]] += -a2;
            m[equationNumber, 2 * columns[3] + 1] += a4;
        }
    }

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)pointList[0].x, (float)pointList[0].y, 0f);
    }
    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        double a1 = pointList[3].y - pointList[2].y + pointList[3].currentDeltas.Item2 - pointList[2].currentDeltas.Item2;
        double a2 = pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2;
        double a3 = pointList[3].x - pointList[2].x + pointList[3].currentDeltas.Item1 - pointList[2].currentDeltas.Item1;
        double a4 = pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1;
        b[startingColumnL, 0] = a4 * a1 - a3 * a2;

        FillRow(new List<int> { 0, 2, 3 }, a, b, startingColumnL, a1, a3, -1.0);
        FillRow(new List<int> { 1, 2, 3 }, a, b, startingColumnL, a1, a3, 1.0);
        FillRow(new List<int> { 2, 0, 1 }, a, b, startingColumnL, a2, a4, 1.0);
        FillRow(new List<int> { 3, 0, 1 }, a, b, startingColumnL, a2, a4, -1.0);
    }

    public void FillRow(List<int> permutation, Matrix<double> a, Matrix<double> b, int startingColumnL,
       double c1, double c2, double multiplier)
    {

        if (columns[permutation[0]] != -1 && !pointList[permutation[0]].isFixed)
        {
            FillDiagonalValue(a, columns[permutation[0]], pointList[permutation[0]]);
            b[2 * columns[permutation[0]], 0] += pointList[permutation[0]].currentDeltas.Item1 + lambdaList[0] * c1 * multiplier;
            b[2 * columns[permutation[0]] + 1, 0] += pointList[permutation[0]].currentDeltas.Item2 - lambdaList[0] * c2 * multiplier;

            a[2 * columns[permutation[0]], startingColumnL] = multiplier * c1;
            a[2 * columns[permutation[0]] + 1, startingColumnL] = -multiplier * c2;

            a[startingColumnL, 2 * columns[permutation[0]]] = multiplier * c1;
            a[startingColumnL, 2 * columns[permutation[0]] + 1] = -multiplier * c2;

            if (columns[permutation[1]] != -1)
            {
                a[2 * columns[permutation[0]], columns[permutation[1]] + 1] += -multiplier * lambdaList[0];
                a[2 * columns[permutation[0]] + 1, columns[permutation[1]]] += multiplier * lambdaList[0];
            }

            if (columns[permutation[2]] != -1)
            {
                a[2 * columns[permutation[0]], 2 * columns[permutation[2]] + 1] += multiplier * lambdaList[0];
                a[2 * columns[permutation[0]] + 1, 2 * columns[permutation[2]]] += -multiplier * lambdaList[0];
            }
        }
    }

    public override double EstimateError()
    {
        return Math.Abs((pointList[3].y - pointList[2].y + pointList[3].currentDeltas.Item2 - pointList[2].currentDeltas.Item2) *
            (pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1) - 
            (pointList[3].x - pointList[2].x + pointList[3].currentDeltas.Item1 - pointList[2].currentDeltas.Item1) * 
            (pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2));
    }
}

public class PerpendicularLines : Constraint
{
    public PerpendicularLines(Segment s1, Segment s2) : base(1, new List<Point> { s1.p1, s1.p2, s2.p1, s2.p2 })
    {
        constraintedSegments.Add(s1);
        constraintedSegments.Add(s2);
    }

    public override void FillDerivatives(Matrix<double> m, int equationNumber, Matrix<double> b, int startingColumn = -1)
    {
        double a1 = pointList[3].y - pointList[2].y, a2 = pointList[1].y - pointList[0].y,
            a3 = pointList[3].x - pointList[2].x, a4 = pointList[1].x - pointList[0].x;

        if (!pointList[0].IsOrigin())
        {
            m[equationNumber, 2 * columns[0]] += -a1;
            m[equationNumber, 2 * columns[0] + 1] += a3;
        }

        if (!pointList[1].IsOrigin())
        {
            m[equationNumber, 2 * columns[1]] += a1;
            m[equationNumber, 2 * columns[1] + 1] += -a3;
        }

        if (!pointList[2].IsOrigin())
        {
            m[equationNumber, 2 * columns[2]] += a2;
            m[equationNumber, 2 * columns[2] + 1] += -a4;
        }

        if (!pointList[3].IsOrigin())
        {
            m[equationNumber, 2 * columns[3]] += -a2;
            m[equationNumber, 2 * columns[3] + 1] += a4;
        }
    }

    public override void DrawConstraint()
    {
        graphic.gameObject.transform.position = new Vector3((float)pointList[0].x, (float)pointList[0].y, 0f);
    }
    public override void FillJacobian(Matrix<double> a, Matrix<double> b, int startingColumnL, int startingColumnP = -1)
    {
        double a1 = pointList[3].y - pointList[2].y + pointList[3].currentDeltas.Item2 - pointList[2].currentDeltas.Item2;
        double a2 = pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2;
        double a3 = pointList[3].x - pointList[2].x + pointList[3].currentDeltas.Item1 - pointList[2].currentDeltas.Item1;
        double a4 = pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1;
        b[startingColumnL, 0] = a4 * a1 - a3 * a2;

        FillRow(new List<int> { 0, 2, 3 }, a, b, startingColumnL, a1, a3, -1.0);
        FillRow(new List<int> { 1, 2, 3 }, a, b, startingColumnL, a1, a3, 1.0);
        FillRow(new List<int> { 2, 0, 1 }, a, b, startingColumnL, a2, a4, 1.0);
        FillRow(new List<int> { 3, 0, 1 }, a, b, startingColumnL, a2, a4, -1.0);
    }

    public void FillRow(List<int> permutation, Matrix<double> a, Matrix<double> b, int startingColumnL,
       double c1, double c2, double multiplier)
    {

        if (columns[permutation[0]] != -1 && !pointList[permutation[0]].isFixed)
        {
            FillDiagonalValue(a, columns[permutation[0]], pointList[permutation[0]]);
            b[2 * columns[permutation[0]], 0] += pointList[permutation[0]].currentDeltas.Item1 + lambdaList[0] * c1 * multiplier;
            b[2 * columns[permutation[0]] + 1, 0] += pointList[permutation[0]].currentDeltas.Item2 - lambdaList[0] * c2 * multiplier;

            a[2 * columns[permutation[0]], startingColumnL] = multiplier * c1;
            a[2 * columns[permutation[0]] + 1, startingColumnL] = -multiplier * c2;

            a[startingColumnL, 2 * columns[permutation[0]]] = multiplier * c1;
            a[startingColumnL, 2 * columns[permutation[0]] + 1] = -multiplier * c2;

            if (columns[permutation[1]] != -1)
            {
                a[2 * columns[permutation[0]], columns[permutation[1]] + 1] += -multiplier * lambdaList[0];
                a[2 * columns[permutation[0]] + 1, columns[permutation[1]]] += multiplier * lambdaList[0];
            }

            if (columns[permutation[2]] != -1)
            {
                a[2 * columns[permutation[0]], 2 * columns[permutation[2]] + 1] += multiplier * lambdaList[0];
                a[2 * columns[permutation[0]] + 1, 2 * columns[permutation[2]]] += -multiplier * lambdaList[0];
            }
        }
    }

    public override double EstimateError()
    {
        return Math.Abs((pointList[3].y - pointList[2].y + pointList[3].currentDeltas.Item2 - pointList[2].currentDeltas.Item2) *
            (pointList[1].x - pointList[0].x + pointList[1].currentDeltas.Item1 - pointList[0].currentDeltas.Item1) -
            (pointList[3].x - pointList[2].x + pointList[3].currentDeltas.Item1 - pointList[2].currentDeltas.Item1) *
            (pointList[1].y - pointList[0].y + pointList[1].currentDeltas.Item2 - pointList[0].currentDeltas.Item2));
    }
}