using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using DG.Tweening;

public class GroundMeshCreator : MonoBehaviour
{
    [Header("Common settings")]

    [SerializeField]
    Vector2 m_cellSize = new Vector2(0.1f, 0.1f);

    [SerializeField]
    Vector2 m_fieldSizeX = new Vector2(-4.0f, 4.0f);

    [SerializeField]
    Vector2 m_fieldSizeY = new Vector2(-5.0f, 5.0f);

    [SerializeField]
    float m_nearestNodeSignificantDistance = 0.5f;
    float m_sqrNearestNodeSignificantDistance = 1.0f;

    [SerializeField]
    float m_nearestNodeMaxDistance = 1.0f;
    float m_sqrNearestNodeMaxDistance = 1.0f;

    [SerializeField]
    int m_searchDepth = 2;

    [Header("Grass Settings")]

    [SerializeField]
    GameObject m_borderPrefab = null;

    [SerializeField]
    GameObject m_innerBorderPrefab = null;

    [SerializeField]
    GameObject m_outerBorderPrefab = null;

    [SerializeField]
    GameObject m_borderContent = null;

    [SerializeField]
    float m_radius;
    float m_sqrRadius;

    [SerializeField]
    float m_negativeRadius;
    float m_sqrNegativeRadius;

    [SerializeField]
    float m_influenceDistance = 1.0f;
    float m_sqrInfluenceDistance;

    [SerializeField]
    float m_influenceMultiplier = 1.0f;

    [SerializeField]
    float m_factor = 1.0f;

    [SerializeField]
    float m_groundInnerBorderShift = -0.05f;

    [SerializeField]
    float m_groundOuterBorderShift = -0.02f;

    [Header("Predefined data")]
    /** Predefined values which are calculated every ground propagation. */
    float[] m_predefinedValues;

    /** Predefined calculations of distance etc. Calculated once for topology. */
    PointData[] m_predefinedPointData;

    bool[] m_nodeGroundFlags;

    [Header("Variables")]

    Vector2Int m_numberOfCells;
    List<GameNode> m_nodeList = null;

    List<GameObject> m_groundBorder = new List<GameObject>();
    float m_groundBorderWidth = 1.0f;

    Mesh m_mesh = null;

    delegate float GetValueMethod(int index);
    delegate void PrepareMethod();

    enum Turn
    {
        LEFT,
        RIGHT,
        NONE
    }

    struct PointData
    {
        public int[] influenceNodes;
        public float[] values;
        public Vector2 point;
        public bool excludePoint;
        public bool isSignificantInfluenced;
    }

    struct VertexData
    {
        public VertexData(Vector2 point, Vector2 normal)
        {
            this.point = point;
            this.normal = normal;
        }

        public Vector2 point;
        public Vector2 normal;
    }

    /**
     * \brief An edge pointed so that the ground is on the right side.
     *        Direction is from point `a` to point `b`.
     **/
    class Edge
    {
        public Vector2 a;
        public Vector2 b;

        public int indexA; //!< Start index.
        public int indexB; //!< End index

        /** Turn tells us whether we're going left or right. */
        public Turn turn = Turn.NONE;

        public Edge(Vector2 a, Vector2 b)
        {
            this.a = a;
            this.b = b;
        }

        public Edge(Vector2 a, Vector2 b, int indexA, int indexB, Turn turn)
        {
            this.a = a;
            this.b = b;
            this.indexA = indexA;
            this.indexB = indexB;
            this.turn = turn;
        }
    }

    void Awake()
    {
        var currentNodes = FindObjectsOfType<GameNode>(true);
        createGround(currentNodes.ToList());
        // Initialize();
    }

    void Initialize()
    {
        // m_borderPrefab.GetComponent<SpriteRenderer>()
        m_groundBorderWidth = 0.2f; //0.9f * m_borderPrefab.GetComponent<tk2dSprite>().GetBounds().size.x; // 0.2f!
        GetComponent<MeshFilter>().sharedMesh = m_mesh = new Mesh();
        m_mesh.name = "GroundMesh";
        createSquaredVariables();
        prepareGrid();
    }

    #region helper methods

    void createSquaredVariables()
    {
        m_sqrRadius = m_radius * m_radius;
        m_sqrNegativeRadius = m_negativeRadius * m_negativeRadius;
        m_sqrInfluenceDistance = m_influenceDistance * m_influenceDistance;
        m_sqrNearestNodeMaxDistance = m_nearestNodeMaxDistance * m_nearestNodeMaxDistance;
        m_sqrNearestNodeSignificantDistance = m_nearestNodeSignificantDistance * m_nearestNodeSignificantDistance;
    }

    List<int> findNodes(GameNode node, List<GameNode> nodeList)
    {
        var nodes = new List<GameNode> {node};
        var openedNodes = new List<System.Tuple<GameNode, int>> {System.Tuple.Create(node, 0)};
        node.isVisited = true;

        while (openedNodes.Count != 0)
        {
            var currentTuple = openedNodes.First();
            openedNodes.RemoveAt(0);

            foreach (var chainId in currentTuple.Item1.chainElements)
            {
                var adjacentNode = nodeList.Find(x => x.id == chainId);

                if (adjacentNode != null && !adjacentNode.isVisited)
                {
                    adjacentNode.isVisited = true;
                    nodes.Add(adjacentNode);

                    if (currentTuple.Item2 + 1 < m_searchDepth)
                    {
                        openedNodes.Add(System.Tuple.Create(adjacentNode, currentTuple.Item2 + 1));
                    }
                }
            }
        }

        nodes.ForEach(x => x.isVisited = false);
        return nodes.ConvertAll(x => x.id);
    }

    GameNode findNearestNode(Vector2 point, List<GameNode> nodeList)
    {
        var minDistance = m_sqrNearestNodeMaxDistance;
        GameNode nearestNode = null;

        foreach (var node in nodeList)
        {
            var distance = (point - (Vector2)node.transform.position).sqrMagnitude;

            if (distance < minDistance)
            {
                minDistance = distance;
                nearestNode = node;
            }
        }

        return nearestNode;
    }

    #endregion

    #region algorithm

    bool isSignificantInfluenced(PointData pointData, Vector2[] nodePositions)
    {
        var nodePosition = nodePositions[pointData.influenceNodes[0]];
        var point = pointData.point;
        return dot(nodePosition.x, point.x, nodePosition.y, point.y) < m_sqrNearestNodeSignificantDistance;
    }

    float[] calculateInfluence(PointData pointData, Vector2[] nodePositions)
    {
        var values = new float[pointData.influenceNodes.Length];
        Vector2 point = pointData.point;

        for (var i = 0; i < values.Length; ++i)
        {
            Vector2 position = nodePositions[pointData.influenceNodes[i]];
            var sqrDistance = dot(position.x, point.x, position.y, point.y);
            //float coeff = (sqrDistance <= m_sqrInfluenceDistance) ? 1.0f : 1.0f / Mathf.Exp(sqrDistance - m_sqrInfluenceDistance);
            float coeff = m_influenceMultiplier / Mathf.Exp(sqrDistance - m_sqrInfluenceDistance);
            values[i] = coeff / sqrDistance;
        }

        return values;
    }

    float calculateValue(PointData pointData)
    {
        float value = 0.0f;

        // if (pointData.isSignificantInfluenced && !m_nodeGroundFlags[pointData.influenceNodes[0]])
        // 	return value;

        for (var i = 0; i < pointData.influenceNodes.Length; ++i)
        {
            value += m_nodeGroundFlags[pointData.influenceNodes[i]] ? m_sqrRadius * pointData.values[i] : -m_sqrNegativeRadius * pointData.values[i];
        }

        return value;
    }

    float getValue(int index)
    {
        return (index < m_predefinedValues.Length) ? m_predefinedValues[index] : 0.0f;
        // return m_predefinedValues[index];
    }

    float dot(float x1, float x2, float y1, float y2)
    {
        var x = x1 - x2;
        var y = y1 - y2;
        return x * x + y * y;
    }

    float GetLerp(float a, float b, float value)
    {
        return (value - a) / (b - a);
    }

    List<Edge> getEdges(int index, Vector2 position, Vector2 size, float range, GetValueMethod getValue)
    {
        var av = position;
        var bv = position + new Vector2(0, size.y);
        var cv = position + new Vector2(size.x, 0);
        var dv = position + size;
        int rowNumber = m_numberOfCells.y;
        float a = getValue(index);
        float b = getValue(index + 1);
        float c = getValue(index + rowNumber);
        float d = getValue(index + rowNumber + 1);
        var edges = new List<Edge>();

        if (a > range)
        {
            if (b > range)
            {
                if (c > range)
                {
                    if (d > range)
                    {
                        m_predefinedPointData[index].excludePoint = true;
                        return null;
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(dv, bv, GetLerp(d, b, range)),
                            Vector2.Lerp(dv, cv, GetLerp(d, c, range)),
                            index + 1,
                            index + rowNumber,
                            Turn.LEFT);
                        edges.Add(edge);
                    }
                }
                else
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(cv, dv, GetLerp(c, d, range)),
                            Vector2.Lerp(cv, av, GetLerp(c, a, range)),
                            index + rowNumber,
                            index - 1,
                            Turn.LEFT);
                        edges.Add(edge);
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(dv, bv, GetLerp(d, b, range)),
                            Vector2.Lerp(cv, av, GetLerp(c, a, range)),
                            index + 1,
                            index - 1,
                            Turn.NONE);
                        edges.Add(edge);
                    }
                }
            }
            else
            {
                if (c > range)
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(bv, av, GetLerp(b, a, range)),
                            Vector2.Lerp(bv, dv, GetLerp(b, d, range)),
                            index - rowNumber,
                            index + 1,
                            Turn.LEFT);
                        edges.Add(edge);
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(bv, av, GetLerp(b, a, range)),
                            Vector2.Lerp(dv, cv, GetLerp(d, c, range)),
                            index - rowNumber,
                            index + rowNumber,
                            Turn.NONE);
                        edges.Add(edge);
                    }
                }
                else
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(bv, av, GetLerp(b, a, range)),
                            Vector2.Lerp(cv, av, GetLerp(c, a, range)),
                            index - rowNumber,
                            index - 1,
                            Turn.RIGHT);
                        edges.Add(edge);
                        edge = new Edge(
                            Vector2.Lerp(cv, dv, GetLerp(c, d, range)),
                            Vector2.Lerp(bv, dv, GetLerp(b, d, range)),
                            index + rowNumber,
                            index + 1,
                            Turn.RIGHT);
                        edges.Add(edge);
                        Debug.Log("Cross 1");
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(bv, av, GetLerp(b, a, range)),
                            Vector2.Lerp(cv, av, GetLerp(c, a, range)),
                            index - rowNumber,
                            index - 1,
                            Turn.RIGHT);
                        edges.Add(edge);
                    }
                }
            }
        }
        else
        {
            if (b > range)
            {
                if (c > range)
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(av, cv, GetLerp(a, c, range)),
                            Vector2.Lerp(av, bv, GetLerp(a, b, range)),
                            index - 1,
                            index - rowNumber,
                            Turn.LEFT);
                        edges.Add(edge);
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(av, cv, GetLerp(a, c, range)),
                            Vector2.Lerp(av, bv, GetLerp(a, b, range)),
                            index - 1,
                            index - rowNumber,
                            Turn.LEFT);
                        edges.Add(edge);
                        edge = new Edge(
                            Vector2.Lerp(dv, bv, GetLerp(d, b, range)),
                            Vector2.Lerp(dv, cv, GetLerp(d, c, range)),
                            index + 1,
                            index + rowNumber,
                            Turn.LEFT);
                        edges.Add(edge);
                        Debug.Log("Cross 2");
                    }
                }
                else
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(cv, dv, GetLerp(c, d, range)),
                            Vector2.Lerp(av, bv, GetLerp(a, b, range)),
                            index + rowNumber,
                            index - rowNumber,
                            Turn.NONE);
                        edges.Add(edge);
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(dv, bv, GetLerp(d, b, range)),
                            Vector2.Lerp(av, bv, GetLerp(a, b, range)),
                            index + 1,
                            index - rowNumber,
                            Turn.RIGHT);
                        edges.Add(edge);
                    }
                }
            }
            else
            {
                if (c > range)
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(av, cv, GetLerp(a, c, range)),
                            Vector2.Lerp(bv, dv, GetLerp(b, d, range)),
                            index - 1,
                            index + 1,
                            Turn.NONE);
                        edges.Add(edge);
                    }
                    else
                    {
                        var edge = new Edge(
                            Vector2.Lerp(av, cv, GetLerp(a, c, range)),
                            Vector2.Lerp(dv, cv, GetLerp(d, c, range)),
                            index - 1,
                            index + rowNumber,
                            Turn.RIGHT);
                        edges.Add(edge);
                    }
                }
                else
                {
                    if (d > range)
                    {
                        var edge = new Edge(
                            Vector2.Lerp(cv, dv, GetLerp(c, d, range)),
                            Vector2.Lerp(bv, dv, GetLerp(b, d, range)),
                            index + rowNumber,
                            index + 1,
                            Turn.RIGHT);
                        edges.Add(edge);
                    }
                    else
                    {
                        return null;
                    }
                }
            }
        }

        return edges;
    }

    List<KeyValuePair<bool, List<Vector2>>> getContourSequence(float factor, PrepareMethod prepareMethod, GetValueMethod getValueMethod)
    {
        prepareMethod?.Invoke();
        Dictionary<int, List<Edge>> cellList = new Dictionary<int, List<Edge>>();

        for (var i = 0; i < m_predefinedPointData.Length; ++i)
        {
            if (m_predefinedPointData[i].excludePoint)
            {
                continue;
            }

            var edges = getEdges(i, m_predefinedPointData[i].point, m_cellSize, factor, getValueMethod);

            if (edges != null)
            {
                cellList.Add(i, edges);
            }
        }

        List<KeyValuePair<bool, List<Vector2>>> points = new List<KeyValuePair<bool, List<Vector2>>>();

        while (cellList.Count != 0)
        {
            points.Add(getPoints(cellList));
        }

        return points;
    }

    KeyValuePair<bool, List<Vector2>> getPoints(Dictionary<int, List<Edge>> cellList)
    {
        var cell = cellList.First();
        var firstNode = cell.Key;
        var lastNode = cell.Value.First().indexA;
        var node = firstNode;
        var points = new List<Vector2>();
        int direction = 0;

        while (true)
        {
            List<Edge> edges = null;

            if (!cellList.TryGetValue(node, out edges))
            {
                break;
            }

            var edge = edges.Find(x => x.indexA == lastNode);

            if (edge == null)
            {
                Debug.Log("edge == null!");
            }

            edges.Remove(edge);

            if (edges.Count == 0)
            {
                cellList.Remove(node);
            }

            points.Add(edge.a);
            var nextNode = edge.indexB;

            if (edge.turn != Turn.NONE)
            {
                direction += (edge.turn == Turn.RIGHT) ? 1 : -1;
            }

            lastNode = node;

            if (nextNode == firstNode)
            {
                break;
            }

            node = nextNode;
        }

        return new KeyValuePair<bool, List<Vector2>>(direction >= 0, points);
    }

    #endregion

    #region Preparing & Drawing

    public void createGround()
    {
        //var topologyController = GameComponentsLocator.Get<LevelTopologyController>();
        //createGround(topologyController.nodeList, topologyController.layout.topologyShape);
    }

    public void createGround(List<GameNode> nodeList)
    {
        Initialize(); // should be only once

        m_nodeList = nodeList;
        // Create ground.
        preparePointData();
        prepareGroundNodes();
        //updateGround(new List<GameNode>());
        updateGround(nodeList.FindAll(x => x.hasGround));
    }

    public void updateGround(List<GameNode> nodes)
    {
        if (nodes.Count == 0)
        {
            return;
        }

        // m_groundBorder.ForEach(x => x.Deactive());
        // m_groundBorder.Clear();
        prepareGroundNodes(nodes);
        // StartCoroutine(updateGroundBack(nodes));
        var shapeData = getShape(m_factor, prepareValues, getValue);
        var shape = shapeData;
        // var shape = shapeData & m_topologyShape;
        Core.Utils.ShapeCreator.CreateMesh(m_mesh, shape.Outline(-0.03f), Color.green, false);
        drawGroundBorder(shape);

        nodes.ForEach(x => x.isChecked = true);
    }

    /*IEnumerator updateGroundBack(List<GameNode> nodes)
    {
        yield return new WaitForSeconds(m_meshAppearDelay);
        nodes.ForEach(x => x.activeGround.animateBackAppearance());
    }*/

    Core.Shape getShape(float factor, PrepareMethod prepareMethod, GetValueMethod getValue)
    {
        var points = getContourSequence(m_factor, prepareMethod, getValue);
        var shape = new Core.Shape();

        foreach (var pointGroup in points)
        {
            var newShape = new Core.Shape(pointGroup.Value);
            shape = (pointGroup.Key) ? shape + newShape : shape - newShape;
        }

        return shape;
    }

    void drawGroundBorder(Core.Shape shape)
    {
        var shapeContourData = shape.contourData;

        var children = m_borderContent.transform.GetComponentsInChildren<SpriteRenderer>();
        children.ToList().ForEach(x => DestroyImmediate(x.gameObject));
        m_groundBorder.Clear();

        foreach (var contourData in shapeContourData)
        {
            var borderShift = contourData.Item1 ? m_groundOuterBorderShift : m_groundInnerBorderShift;
            var vertexData = createVertexData(contourData.Item2, borderShift);

            foreach (var vertex in vertexData)
            {
                var point2d = vertex.point + borderShift * vertex.normal.normalized;
                var point = new Vector3(point2d.x, point2d.y, 0.0f);
                var normal = new Vector3(vertex.normal.x, vertex.normal.y, 1.0f);
                var grassBorder = contourData.Item1 ?
                    GameObject.Instantiate<GameObject>(m_outerBorderPrefab) : GameObject.Instantiate<GameObject>(m_innerBorderPrefab);
                grassBorder.transform.SetParent(m_borderContent.transform);
                grassBorder.transform.localPosition = point;
                grassBorder.transform.localRotation = Quaternion.LookRotation(Vector3.back, normal);
                m_groundBorder.Add(grassBorder);
            }
        }
    }

    List<VertexData> createVertexData(List<Vector2> contour, float borderShift)
    {
        var vertexList = new List<VertexData>();
        var borderDistance = m_groundBorderWidth * m_groundBorderWidth / (m_groundBorderWidth - 2 * borderShift / Mathf.Tan(Mathf.PI / 6.0f));
        var remainder = 0.0f;

        for (var i = 1; i < contour.Count; ++i)
        {
            vertexList.AddRange(createVertexData(contour[i], contour[i - 1], borderDistance, ref remainder));
        }

        vertexList.AddRange(createVertexData(contour.First(), contour.Last(), borderDistance, ref remainder));
        vertexList.Add(createFirstVertexData(contour.First(), contour.Last()));
        return vertexList;
    }

    VertexData createFirstVertexData(Vector2 pointA, Vector2 pointB)
    {
        var normal = new Vector2(pointA.y - pointB.y, pointB.x - pointA.x);
        return new VertexData(pointA, normal);
    }

    List<VertexData> createVertexData(Vector2 pointA, Vector2 pointB, float borderDistance, ref float remainder)
    {
        var segment = pointA - pointB;
        var segmentLength = segment.magnitude;
        var length = segmentLength + remainder;
        var vertexData = new List<VertexData>(1);

        if (length > borderDistance)
        {
            var normal = new Vector2(pointA.y - pointB.y, pointB.x - pointA.x);
            var direction = segment / segmentLength;
            var point = pointB + (borderDistance - remainder) * direction;
            vertexData.Add(new VertexData(point, normal));
            length -= borderDistance;

            while (length > m_groundBorderWidth)
            {
                point += m_groundBorderWidth * direction;
                vertexData.Add(new VertexData(point, normal));
                length -= m_groundBorderWidth;
            }
        }

        remainder = length;
        return vertexData;
    }

    // List<VertexData> createVertexData(Vector2 pointA, Vector2 pointB, ref float remainder)
    // {
    //     var direction = pointB - pointA;
    //     var parts = Mathf.CeilToInt(direction.magnitude / m_groundBorderWidth);

    //     var vertexData = new List<VertexData> (parts);
    //     var normal = new Vector2(pointA.y - pointB.y, pointB.x - pointA.x);

    //     for (var i = 0; i < parts; ++i)
    //     {
    //         var alpha = 1.0f / (parts + 1) * (i + 1);
    //         var point = pointA + alpha * direction;
    //         vertexData.Add(new VertexData(point, normal));
    //     }

    //     return vertexData;
    // }

    void prepareGrid()
    {
        var cellsNumberX = Mathf.CeilToInt((m_fieldSizeX.y - m_fieldSizeX.x) / m_cellSize.x) + 1;
        var cellsNumberY = Mathf.CeilToInt((m_fieldSizeY.y - m_fieldSizeY.x) / m_cellSize.y) + 1;
        m_numberOfCells = new Vector2Int(cellsNumberX, cellsNumberY);
    }

    void prepareGroundNodes(List<GameNode> nodes)
    {
        foreach (var node in nodes)
        {
            m_nodeGroundFlags[node.id] = true;
        }
    }

    void prepareGroundNodes()
    {
        foreach (var node in m_nodeList)
        {
            m_nodeGroundFlags[node.id] = node.hasGround; //node.activeGround != null;
        }
    }

    void preparePointData()
    {
        m_predefinedPointData = new PointData[m_numberOfCells.x * m_numberOfCells.y];
        m_predefinedValues = new float[m_predefinedPointData.Length];
        var nodePositions = getNodePositions(m_nodeList);
        m_nodeGroundFlags = new bool[nodePositions.Length];
        int i = 0;
        var point = Vector2.zero;
        for (var x = m_fieldSizeX.x; x < m_fieldSizeX.y; x += m_cellSize.x)
        {
            for (var y = m_fieldSizeY.x; y < m_fieldSizeY.y; y += m_cellSize.y, ++i)
            {
                point.x = x;
                point.y = y;
                var nearestNode = findNearestNode(point, m_nodeList);
                var pointData = new PointData();
                pointData.point = point;
                if (nearestNode != null)
                {
                    pointData.influenceNodes = findNodes(nearestNode, m_nodeList).ToArray();
                    pointData.values = calculateInfluence(pointData, nodePositions);
                    pointData.isSignificantInfluenced = isSignificantInfluenced(pointData, nodePositions);
                }
                else
                {
                    pointData.excludePoint = true;
                }
                m_predefinedPointData[i] = pointData;
            }
        }
    }

    Vector2[] getNodePositions(List<GameNode> nodeList)
    {
        var nodePositions = new Vector2[nodeList.Max(x => x.id) + 1];

        foreach (var node in nodeList)
        {
            nodePositions[node.id] = node.position;
        }

        return nodePositions;
    }

    void prepareValues()
    {
        for (var i = 0; i < m_predefinedValues.Length; ++i)
        {
            if (m_predefinedPointData[i].excludePoint)
            {
                continue;
            }

            m_predefinedValues[i] = calculateValue(m_predefinedPointData[i]);
        }
    }

    #endregion

    #region Gizmos

    void prepareGroundNodesForGizmos()
    {
        if (Application.isPlaying)
        {
            prepareGroundNodes();
            return;
        }

        foreach (var node in m_nodeList)
        {
            m_nodeGroundFlags[node.id] = node.groundType != GameGroundType.NONE;
        }
    }

    void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            return;
        }

        // m_nodeList = FindObjectsOfType<GameNode>().ToList();
        // createSquaredVariables();
        // prepareGrid();
        // drawGizmosGround();
    }

    void drawGizmosGround()
    {
        preparePointData();
        prepareGroundNodesForGizmos();
        var points = getContourSequence(m_factor, prepareValues, getValue);
        drawGizmos(points);
        drawGizmos(m_predefinedPointData);
    }

    void drawGizmos(List<KeyValuePair<bool, List<Vector2>>> points)
    {
        Gizmos.color = Color.yellow;

        foreach (var pointsGroup in points)
        {
            var pointsList = pointsGroup.Value;

            for (var i = 0; i < pointsList.Count - 1; ++i)
            {
                Gizmos.DrawLine(pointsList[i], pointsList[i + 1]);
            }

            Gizmos.DrawLine(pointsList.Last(), pointsList.First());
        }
    }

    void drawGizmos(PointData[] pointDatas)
    {
        Gizmos.color = Color.red;

        foreach (var pointData in pointDatas)
        {
            if (pointData.excludePoint)
            {
                Gizmos.DrawSphere(pointData.point, 0.05f);
            }
        }
    }

    #endregion
}
