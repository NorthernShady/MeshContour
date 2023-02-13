using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Core
{
    namespace Utils
    {
        public static class ShapeCreator
        {
            public static Mesh CreateMesh(Shape innerShape, Color color, bool withAntialiasing, float antialiasingDistance = 0.1f, float antialiasingAlpha = 0.1f)
            {
                return CreateMesh(new Mesh(), innerShape, color, withAntialiasing, antialiasingDistance, antialiasingAlpha);
            }

            public static Mesh CreateMesh(Mesh mesh, Shape innerShape, Color color, bool withAntialiasing, float antialiasingDistance = 0.1f, float antialiasingAlpha = 0.1f)
            {
                var tess = innerShape.Tesselate(color, withAntialiasing, antialiasingDistance, antialiasingAlpha);
                var verticesCount = tess.VertexCount;
                var trianglesCount = tess.ElementCount;
                var triangles = tess.Elements;
                var vertices = new Vector3[verticesCount];
                var colors = new Color[verticesCount];
                Vector2[] myUVs = new Vector2[verticesCount];

                for (int i = 0; i < verticesCount; i++)
                {
                    vertices[i] = tess.Vertices[i].Position.ToVector2D();
                    colors[i] = (Color)tess.Vertices[i].Data;
                    myUVs[i] = vertices[i];
                }

                mesh.Clear();
                mesh.vertices = vertices;
                mesh.colors = colors;
                mesh.triangles = triangles;
                mesh.uv = myUVs;
                mesh.RecalculateBounds();
                return mesh;
            }
        }

        public static class Casting
        {
            public const float INT_SCALE = 100;

            public static ClipperLib.IntPoint ToClipperVector(this Vector2 point)
            {
                return new ClipperLib.IntPoint(point.x * INT_SCALE, point.y * INT_SCALE);
            }

            public static Vector2 ToVector2D(this ClipperLib.IntPoint point)
            {
                return new Vector2(point.X / INT_SCALE, point.Y / INT_SCALE);
            }

            public static ClipperLib.IntPoint ToClipperVector(this LibTessDotNet.Vec3 point)
            {
                return new ClipperLib.IntPoint(point.X, point.Y);
            }

            public static LibTessDotNet.Vec3 ToVectorTess(this ClipperLib.IntPoint point)
            {
                return new LibTessDotNet.Vec3 { X = point.X, Y = point.Y, Z = 0 };
            }

            public static Vector2 ToVector2D(this LibTessDotNet.Vec3 point)
            {
                return new Vector2(point.X / INT_SCALE, point.Y / INT_SCALE);
            }

            public static LibTessDotNet.Vec3 ToVectorTess(this Vector2 point)
            {
                return new LibTessDotNet.Vec3 { X = point.x * INT_SCALE, Y = point.y * INT_SCALE, Z = 0 };
            }
        }
    }
}
