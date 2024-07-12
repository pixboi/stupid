using stupid.Maths;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }
        public Vector3S halfSize { get; private set; }
        public Vector3S[] vertices { get; private set; }
        public Vector3S[] axes { get; private set; }
        public f32[] projections { get; private set; }

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
            this.vertices = new Vector3S[8];
            this.axes = new Vector3S[3];
            this.projections = new f32[3];
        }

        public override bool NeedsRotationUpdate => true;
        public override void OnRotationUpdate()
        {
            UpdateBox();
        }

        public void UpdateBox()
        {
            var rotMat = this.attachedCollidable.transform.rotationMatrix;
            var position = this.attachedCollidable.transform.position;
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

            projections[0] = ProjectBox(halfSize, axes[0], rotMat);
            projections[1] = ProjectBox(halfSize, axes[1], rotMat);
            projections[2] = ProjectBox(halfSize, axes[2], rotMat);
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis, Matrix3S rotation)
        {
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotation.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotation.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotation.GetColumn(2), axis));
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

        public override bool Intersects(Collidable other, out ContactS contact)
        {
            contact = new ContactS();

            if (other.collider is BoxColliderS otherBox)
            {
                return CollisionMath.BoxVsBox(this, otherBox, out contact);
            }

            if (other.collider is SphereColliderS otherSphere)
            {
                return CollisionMath.BoxVsSphere(this, otherSphere, out contact);
            }

            return false;
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
}
