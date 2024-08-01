using stupid.Maths;
using System.Numerics;
using System;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }
        public Vector3S halfSize { get; private set; }
        public Vector3S[] vertices { get; private set; }
        public Vector3S[] axes { get; private set; }
        //public int[] edges { get; private set; }
        //public int[] triangles { get; private set; }

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
            this.vertices = new Vector3S[8];
            this.axes = new Vector3S[3];
            //this.edges = new int[12];
            //this.triangles = new int[6];

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

            // Calculate vertices
            vertices[0] = position + right + up + forward;
            vertices[1] = position + right + up - forward;
            vertices[2] = position + right - up + forward;
            vertices[3] = position + right - up - forward;
            vertices[4] = position - right + up + forward;
            vertices[5] = position - right + up - forward;
            vertices[6] = position - right - up + forward;
            vertices[7] = position - right - up - forward;

            // Calculate axes
            axes[0] = rotMat.GetColumn(0);
            axes[1] = rotMat.GetColumn(1);
            axes[2] = rotMat.GetColumn(2);

            //Put edges and triangles, edge has 2 stride, triangle 3
        }

        public bool ContainsPoint(Vector3S point)
        {
            var localPoint = collidable.transform.ToLocalPoint(point);

            if (localPoint.x > halfSize.x || localPoint.x < -halfSize.x) return false;
            if (localPoint.y > halfSize.y || localPoint.y < -halfSize.y) return false;
            if (localPoint.z > halfSize.z || localPoint.z < -halfSize.z) return false;

            return true;
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

        static readonly f32 boxInertia = f32.FromFloat(0.08333333333f);
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

        public Vector3S SupportFunction(Vector3S direction)
        {
            // Transform the direction to local space
            Vector3S localDirection = collidable.transform.InverseTransformDirection(direction);

            // Calculate the furthest point in the direction of localDirection
            Vector3S localSupportPoint = new Vector3S(
                localDirection.x >= f32.zero ? halfSize.x : -halfSize.x,
                localDirection.y >= f32.zero ? halfSize.y : -halfSize.y,
                localDirection.z >= f32.zero ? halfSize.z : -halfSize.z
            );

            // Transform the support point back to world space
            return collidable.transform.ToWorldPoint(localSupportPoint);
        }


        public Vector3S GetIntersectionPoint(Vector3S worldDirection)
        {
            // Transform the direction to local space
            Vector3S localDirection = collidable.transform.InverseTransformDirection(worldDirection);

            // Normalize the direction vector
            localDirection.Normalize();

            // Origin is the center of the box in local coordinates (0,0,0)
            Vector3S localOrigin = Vector3S.zero;

            // Initialize t values for each face
            f32 tMin = f32.minValue;
            f32 tMax = f32.maxValue;

            // Calculate t values for x-axis
            if (localDirection.x != f32.zero)
            {
                f32 tx1 = (halfSize.x - localOrigin.x) / localDirection.x;
                f32 tx2 = (-halfSize.x - localOrigin.x) / localDirection.x;
                tMin = MathS.Max(tMin, MathS.Min(tx1, tx2));
                tMax = MathS.Min(tMax, MathS.Max(tx1, tx2));
            }

            // Calculate t values for y-axis
            if (localDirection.y != f32.zero)
            {
                f32 ty1 = (halfSize.y - localOrigin.y) / localDirection.y;
                f32 ty2 = (-halfSize.y - localOrigin.y) / localDirection.y;
                tMin = MathS.Max(tMin, MathS.Min(ty1, ty2));
                tMax = MathS.Min(tMax, MathS.Max(ty1, ty2));
            }

            // Calculate t values for z-axis
            if (localDirection.z != f32.zero)
            {
                f32 tz1 = (halfSize.z - localOrigin.z) / localDirection.z;
                f32 tz2 = (-halfSize.z - localOrigin.z) / localDirection.z;
                tMin = MathS.Max(tMin, MathS.Min(tz1, tz2));
                tMax = MathS.Min(tMax, MathS.Max(tz1, tz2));
            }

            // If tMax < tMin, no intersection exists
            if (tMax < tMin)
            {
                return new Vector3S(); // No intersection, return a default value
            }

            // Calculate the intersection point in local space using tMin (since we want the first intersection point)
            Vector3S localIntersectionPoint = localOrigin + localDirection * tMax;

            // Transform the point back to world space
            return collidable.transform.ToWorldPoint(localIntersectionPoint);
        }
    }

}
