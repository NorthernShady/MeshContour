using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ClipperLib;
using Core.Utils;
using ShapeContourData = System.Tuple<bool, System.Collections.Generic.List<UnityEngine.Vector2>>;

namespace Core
{
    public class Shape
    {
        protected List<Vector2> m_vectorPoints;
        List<List<IntPoint>> points;

        public Shape()
        {
            this.points = new List<List<IntPoint>>();
        }

        public Shape(List<Vector2> points)
        {
            m_vectorPoints = points;

            if (Clipper.Area(ToInt(points)) < 0)
            {
                points.Reverse();
            }

            this.points = new List<List<IntPoint>> (1);
            this.points.Add(ToInt(points));
        }

        public List<Vector2> vectorPoints
        {
            get
            {
                return m_vectorPoints;
            }
        }

        public List<List<Vector2>> vectorPointGroups
        {
            get
            {
                return ToVector2(this.points);
            }
        }

        public List<ShapeContourData> contourData
        {
            get
            {
                return ToContourData(this.points);
            }
        }

        private Shape(Shape other)
        {
            if (other.points != null)
            {
                var count = other.points.Count;
                points = new List<List<IntPoint>> (count);

                for (int i = 0; i < count; ++i)
                {
                    var otherInner = other.points[i];
                    var innerCount = otherInner.Count;
                    var inner = new List<IntPoint> (innerCount);

                    for (int j = 0; j < innerCount; ++j)
                    {
                        inner.Add(new IntPoint(otherInner[j]));
                    }

                    points.Add(inner);
                }
            }
            else
            {
                points = null;
            }
        }

        List<IntPoint> ToInt(List<Vector2> points)
        {
            var count = points.Count;
            var result = new List<IntPoint> (count);

            for (int i = 0; i < count; ++i)
            {
                result.Add(points[i].ToClipperVector());
            }

            return result;
        }

        List<List<Vector2>> ToVector2(List<List<IntPoint>> points)
        {
            var result = new List<List<Vector2>> (points.Count);

            foreach (var pointGroup in points)
            {
                var vectorGroup = new List<Vector2> (pointGroup.Count);

                foreach (var point in pointGroup)
                {
                    vectorGroup.Add(point.ToVector2D());
                }

                result.Add(vectorGroup);
            }

            return result;
        }

        List<ShapeContourData> ToContourData(List<List<IntPoint>> points)
        {
            var result = new List<ShapeContourData> (points.Count);

            foreach (var pointGroup in points)
            {
                var isOuterContour = Clipper.Area(pointGroup) > 0.0;
                result.Add(System.Tuple.Create(isOuterContour, pointGroup.ConvertAll(x => x.ToVector2D())));
            }

            return result;
        }

        public Shape Clone()
        {
            return new Shape(this);
        }

        Shape Outline(float value, EndType type)
        {
            if (points == null)
            {
                return new Shape();
            }

            var result = new Shape();
            var clipper = new ClipperOffset();
            clipper.AddPaths(points, JoinType.jtMiter, type);
            clipper.Execute(ref result.points, value * Casting.INT_SCALE);
            return result;
        }

        public Shape Outline(float value)
        {
            return Outline(value, EndType.etClosedPolygon);
        }

        public Shape OutlineContour(float value)
        {
            return Outline(value, EndType.etClosedLine);
        }

        public void Draw(Color color)
        {
            if (points == null)
            {
                return;
            }

            var realColor = color;

            for (int i = 0, count = points.Count; i < count; ++i)
            {
                Gizmos.color = color;
                var path = points[i];
                color.a = realColor.a * (Clipper.Orientation(path) ? 1.0f : 0.5f);

                for (int j = 0, pathCount = path.Count; j < pathCount; ++j)
                {
                    Gizmos.DrawLine(path[j].ToVector2D(), path[(j + 1) % pathCount].ToVector2D());
                }
            }
        }

        public static Shape operator& (Shape a, Shape b)
        {
            if (b.points == null)
            {
                return a.Clone();
            }

            if (a.points == null)
            {
                return b.Clone();
            }

            Shape result = new Shape();
            var clipper = new Clipper();
            clipper.AddPaths(a.points, PolyType.ptSubject, true);
            clipper.AddPaths(b.points, PolyType.ptClip, true);
            clipper.Execute(ClipType.ctIntersection, result.points);
            return result;
        }

        public static Shape operator- (Shape a, Shape b)
        {
            if (b.points == null)
            {
                return a.Clone();
            }

            if (a.points == null)
            {
                return new Shape();
            }

            Shape result = new Shape();
            var clipper = new Clipper();
            clipper.AddPaths(a.points, PolyType.ptSubject, true);
            clipper.AddPaths(b.points, PolyType.ptClip, true);
            clipper.Execute(ClipType.ctDifference, result.points);
            return result;
        }

        public static Shape operator+ (Shape a, Shape b)
        {
            if (b.points == null)
            {
                return a.Clone();
            }

            if (a.points == null)
            {
                return b.Clone();
            }

            Shape result = new Shape();
            var clipper = new Clipper();
            clipper.AddPaths(a.points, PolyType.ptSubject, true);
            clipper.AddPaths(b.points, PolyType.ptClip, true);
            clipper.Execute(ClipType.ctUnion, result.points);
            return result;
        }

        private static object VertexCombine(LibTessDotNet.Vec3 position, object[] data, float[] weights)
        {
            // Fetch the vertex data.
            var colors = new Color[] { (Color)data[0], (Color)data[1], (Color)data[2], (Color)data[3] };
            // Interpolate with the 4 weights.
            var color = new Color(
                colors[0].r * weights[0] + colors[1].r * weights[1] + colors[2].r * weights[2] + colors[3].r * weights[3],
                colors[0].g * weights[0] + colors[1].g * weights[1] + colors[2].g * weights[2] + colors[3].g * weights[3],
                colors[0].b * weights[0] + colors[1].b * weights[1] + colors[2].b * weights[2] + colors[3].b * weights[3],
                colors[0].a * weights[0] + colors[1].a * weights[1] + colors[2].a * weights[2] + colors[3].a * weights[3]
            );
            // Return interpolated data for the new vertex.
            return color;
        }

        /*public LibTessDotNet.Tess Tesselate(Color color, bool withAntialiasing) {
        	var tess = new LibTessDotNet.Tess();

        	for (int i = 0, count = points.Count; i < count; ++i) {
        		var path = points[i];
        		var contour = new LibTessDotNet.ContourVertex[path.Count];

        		for (int j = 0, pathCount = path.Count; j < pathCount; ++j) {
        			contour[j].Position = path[j].ToVectorTess();
        			contour[j].Data = color;
        		}

        		tess.AddContour(contour);
        	}

        	tess.Tessellate(LibTessDotNet.WindingRule.EvenOdd, LibTessDotNet.ElementType.Polygons, 3, VertexCombine);

        	return tess;
        }*/

        static void AddContours(LibTessDotNet.Tess tess, Shape shape, Color color)
        {
            for (int i = 0, count = shape.points.Count; i < count; ++i)
            {
                var path = shape.points[i];
                var contour = new LibTessDotNet.ContourVertex[path.Count];

                for (int j = 0, pathCount = path.Count; j < pathCount; ++j)
                {
                    contour[j].Position = path[j].ToVectorTess();
                    contour[j].Data = color;
                }

                tess.AddContour(contour);
            }
        }

        public LibTessDotNet.Tess Tesselate(Color color, bool withAntialiasing, float antialiasingDistance = 0.1f, float antialiasingAlpha = 0.1f)
        {
            var tess = new LibTessDotNet.Tess();

            if (withAntialiasing)
            {
                AddContours(tess, Outline(antialiasingDistance), new Color(color.r, color.g, color.b, antialiasingAlpha));
            }

            AddContours(tess, this, color);
            tess.Tessellate(LibTessDotNet.WindingRule.NonZero, LibTessDotNet.ElementType.Polygons, 3, VertexCombine);
            return tess;
        }

        public bool IsEmpty
        {
            get
            {
                return points == null || points.Count == 0;
            }
        }

    }
}
