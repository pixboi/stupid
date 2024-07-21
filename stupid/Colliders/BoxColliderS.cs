using stupid.Maths;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }
        public Vector3S halfSize { get; private set; }
        public Vector3S[] vertices { get; private set; }
        public Vector3S[] axes { get; private set; }
        public EdgeS[] edges { get; private set; }
        public FaceS[] faces { get; private set; }

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
            this.vertices = new Vector3S[8];
            this.axes = new Vector3S[3];
            this.edges = new EdgeS[12];
            this.faces = new FaceS[6];

            if (this.collidable != null)
                UpdateBox();
        }

        public override bool NeedsRotationUpdate => true;
        public override void OnRotationUpdate()
        {
            UpdateBox();
        }

        public void UpdateBox()
        {
            var rotMat = this.collidable.transform.rotationMatrix;
            var position = this.collidable.transform.position;
            Vector3S right = rotMat.GetColumn(0) * halfSize.x;
            Vector3S up = rotMat.GetColumn(1) * halfSize.y;
            Vector3S forward = rotMat.GetColumn(2) * halfSize.z;

            vertices[0] = position + right + up + forward;
            vertices[1] = position + right + up - forward;
            vertices[2] = position + right - up + forward;
            vertices[3] = position + right - up - forward;
            vertices[4] = position - right + up + forward;
            vertices[5] = position - right + up - forward;
            vertices[6] = position - right - up + forward;
            vertices[7] = position - right - up - forward;

            axes[0] = rotMat.GetColumn(0);
            axes[1] = rotMat.GetColumn(1);
            axes[2] = rotMat.GetColumn(2);

            UpdateEdges();
            UpdateFaces();
        }

        private void UpdateEdges()
        {
            edges[0] = new EdgeS(vertices[0], vertices[1]);
            edges[1] = new EdgeS(vertices[0], vertices[2]);
            edges[2] = new EdgeS(vertices[0], vertices[4]);

            edges[3] = new EdgeS(vertices[1], vertices[3]);
            edges[4] = new EdgeS(vertices[1], vertices[5]);

            edges[5] = new EdgeS(vertices[2], vertices[3]);
            edges[6] = new EdgeS(vertices[2], vertices[6]);

            edges[7] = new EdgeS(vertices[3], vertices[7]);

            edges[8] = new EdgeS(vertices[4], vertices[5]);
            edges[9] = new EdgeS(vertices[4], vertices[6]);

            edges[10] = new EdgeS(vertices[5], vertices[7]);

            edges[11] = new EdgeS(vertices[6], vertices[7]);
        }

        private void UpdateFaces()
        {
            // Define the faces using vertices and normals
            faces[0] = new FaceS(vertices[0], vertices[1], vertices[5], vertices[4], axes[2]);
            faces[1] = new FaceS(vertices[2], vertices[3], vertices[7], vertices[6], -axes[2]);
            faces[2] = new FaceS(vertices[0], vertices[2], vertices[6], vertices[4], axes[1]);
            faces[3] = new FaceS(vertices[1], vertices[3], vertices[7], vertices[5], -axes[1]);
            faces[4] = new FaceS(vertices[0], vertices[1], vertices[3], vertices[2], axes[0]);
            faces[5] = new FaceS(vertices[4], vertices[5], vertices[7], vertices[6], -axes[0]);
        }

        public override BoundsS CalculateAABB(Vector3S position, QuaternionS rotation)
        {
            // Get the rotation matrix from the quaternion
            var rotationMatrix = Matrix3S.Rotate(rotation);

            // Calculate the extents of the rotated box along each axis
            Vector3S rotatedHalfSize = new Vector3S(
                MathS.Abs(rotationMatrix.m00) * halfSize.x + MathS.Abs(rotationMatrix.m01) * halfSize.y + MathS.Abs(rotationMatrix.m02) * halfSize.z,
                MathS.Abs(rotationMatrix.m10) * halfSize.x + MathS.Abs(rotationMatrix.m11) * halfSize.y + MathS.Abs(rotationMatrix.m12) * halfSize.z,
                MathS.Abs(rotationMatrix.m20) * halfSize.x + MathS.Abs(rotationMatrix.m21) * halfSize.y + MathS.Abs(rotationMatrix.m22) * halfSize.z
            );

            var min = position - rotatedHalfSize;
            var max = position + rotatedHalfSize;
            _bounds = new BoundsS(min, max);
            return _bounds;
        }

        public override int Intersects(Collidable other, ref ContactS contact)
        {
            if (other.collider is BoxColliderS otherBox)
            {
                return CollisionMath.BoxVsBox(this, otherBox, ref contact);
            }

            if (other.collider is SphereColliderS otherSphere)
            {
                return CollisionMath.BoxVsSphere(this, otherSphere, ref contact);
            }

            return 0;
        }

        static readonly f32 boxInertia = (f32)1 / (f32)12;
        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // For a solid box: I = 1/12 * m * (h^2 + d^2) for each axis
            var h = size.x;
            var d = size.y;
            var w = size.z;
            var inertiaX = boxInertia * mass * (d * d + w * w);
            var inertiaY = boxInertia * mass * (h * h + w * w);
            var inertiaZ = boxInertia * mass * (h * h + d * d);

            return new Matrix3S(
                new Vector3S(inertiaX, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertiaY, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertiaZ)
            );
        }
    }

    public readonly struct EdgeS
    {
        public readonly Vector3S start;
        public readonly Vector3S end;

        public EdgeS(Vector3S start, Vector3S end)
        {
            this.start = start;
            this.end = end;
        }
    }

    public readonly struct FaceS
    {
        public readonly Vector3S vertex1;
        public readonly Vector3S vertex2;
        public readonly Vector3S vertex3;
        public readonly Vector3S vertex4;
        public readonly Vector3S normal;

        public FaceS(Vector3S vertex1, Vector3S vertex2, Vector3S vertex3, Vector3S vertex4, Vector3S normal)
        {
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            this.vertex3 = vertex3;
            this.vertex4 = vertex4;
            this.normal = normal;
        }
    }
}
