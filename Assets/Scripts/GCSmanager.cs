using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class GCSmanager : MonoBehaviour
{
    //список ограничений
    private List<Constraint> constraints;
    //список точек
    private List<Point> points;
    //список отрезков
    private List<Segment> segments;
    //вектор свободных членов, как еще назвать хз
    private List<float> freeVector;
    //иногда 1 ограничение порождает 2 уравнения в системе
    //хз, какой костыль лучше
    public static int equationCount = 0;
    //точка начала координат, характерный id = -1
    [HideInInspector]
    public static Point origin = new Point(0.0f, 0.0f, -1);
    [HideInInspector]
    public static List<Point> constraitedPoints = new List<Point>();
    //костыль, чтобы возвращать уникальные id точек
    public int pointsCreated = 0;

    void Awake()
    {
        constraints = new List<Constraint>();
        points = new List<Point>();
        segments = new List<Segment>();
    }

    void Update()
    {
        
    }

    private void UpdatePoints()
    {

    }

    //проверяет, есть ли эта точка уже в ограничениях
    public static void CheckForPoint(Point p)
    {
       // if (constraitedPoints.Exists(point => point.pointID == p.pointID))
        if (p.columnID == -1) 
        {
            p.columnID = constraitedPoints.Count;
            constraitedPoints.Add(p);
        }
    }


    //функция, которая будет вызываться при создании точки
    public void CreatePoint(float x, float y)
    {
        //points.Add(new Point(x, y, pointsCreated));
        points.Add(new Point(pointsCreated));
        pointsCreated++;
    }


    //ультимативная функция, проверяющая состояние системы
    public bool CheckState()
    {
        if (points.Count == equationCount)
        {
            return true;
        }
        else
        {
            return false;
        }
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

    //для тестов
    public Point(int id) : this(Random.Range(0.0f, 10.0f), Random.Range(0.0f, 10.0f), id) { }
}

//абстрактный класс ограничения
abstract public class Constraint
{
    //возможно надо, чтобы знать, по скольким лямбдам ждать производные
    //а еще - это количество порождаемых уравнений
    readonly int lambdas;
    public Constraint(int lambdas)
    {
        this.lambdas = lambdas;
        GCSmanager.equationCount += lambdas;
    }

    //метод возвращает кортежи частная производная по точке - точка
    //b - свободный член CLAУ

    //возможно разумнее ничего не возвращать,
    //а просто дополнять СЛАУ на месте
    public abstract List<(float, Point)> Derivatives(out float b);

    //Добавляет неактивную только что задействованную точку в список, включенных в СЛАУ
    public void AddPoint(Point p)
    {
        if (p.pointID != -1)
        {
            GCSmanager.CheckForPoint(p);
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
    public Point p1;
    public Point p2;
    public Distance(Point p1, Point p2, float d) : base(1)
    {
        this.p1 = p1;
        this.p2 = p2;
        distance = d;
        AddPoint(p1);
        AddPoint(p2);
    }
    public override List<(float, Point)> Derivatives(out float b)
    {
        List<(float, Point)> ans = new List<(float, Point)>();
        if (p1.pointID != -1)
        {
            ans.Add((2 * (p1.x - p2.x), p1));
            ans.Add((2 * (p1.y - p2.y), p1));
        }
        if (p2.pointID != -1)
        {
            ans.Add((2 * (p2.x - p1.x), p2));
            ans.Add((2 * (p2.y - p1.y), p2));
        }

        b = Mathf.Pow(p1.x - p2.x, 2.0f) + Mathf.Pow(p1.y - p2.y, 2.0f) - Mathf.Pow(distance, 2.0f);
        return ans;
    }
}
