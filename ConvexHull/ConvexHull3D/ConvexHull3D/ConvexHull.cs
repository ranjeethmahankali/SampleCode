using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using Rhino.Geometry;

namespace ConvexHull3D
{
    public static class ConvexHull
    {
        [DllImport("CPPHull.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Unsafe_ComputeHull([MarshalAs(UnmanagedType.LPArray)] double[] ptCoords, ulong nPts, 
            ref IntPtr triangles, ref int nTriangles);

        public static Mesh Create(List<Point3d> points)
        {
            if (points.Count < 4)
            {
                return null;
            }
            double[] coords = points.SelectMany(pt => new double[] { pt.X, pt.Y, pt.Z }).ToArray();
            IntPtr trPtr = IntPtr.Zero;
            int nTriangles = 0;
            Unsafe_ComputeHull(coords, (ulong)points.Count, ref trPtr, ref nTriangles);
            int[] triangleIndices = new int[nTriangles * 3];
            Marshal.Copy(trPtr, triangleIndices, 0, (int)nTriangles * 3);

            Mesh mesh = new Mesh();
            mesh.Vertices.AddVertices(points);
            for (int i = 0; i < nTriangles; i++)
            {
                mesh.Faces.AddFace(triangleIndices[3 * i], triangleIndices[3 * i + 1], triangleIndices[3 * i + 2]);
            }

            // TODO: Compact the mesh to remove points not in the hull.
            return mesh;
        }
    }
}
