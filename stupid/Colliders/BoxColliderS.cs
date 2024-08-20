using stupid.Maths;
using System;

namespace stupid.Colliders
{
    public struct BoxColliderS : IShape
    {
        public readonly Vector3S size, halfSize, right, up, forward;
        public readonly Vector3S[] localVertices;
        public Vector3S[] vertices, axes;

        private Collidable _collidable;
        public Collidable collidable => _collidable;

        private BoundsS _bounds;
        public BoundsS bounds => _bounds;

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
            this.right = new Vector3S(halfSize.x, f32.zero, f32.zero);
            this.up = new Vector3S(f32.zero, halfSize.y, f32.zero);
            this.forward = new Vector3S(f32.zero, f32.zero, halfSize.z);

            this.localVertices = new Vector3S[8];
            localVertices[0] = right + up + forward;
            localVertices[1] = right + up - forward;
            localVertices[2] = right - up + forward;
            localVertices[3] = right - up - forward;
            localVertices[4] = -right + up + forward;
            localVertices[5] = -right + up - forward;
            localVertices[6] = -right - up + forward;
            localVertices[7] = -right - up - forward;

            this.vertices = new Vector3S[8];
            this.axes = new Vector3S[3];
            this._collidable = null;
            this._bounds = new BoundsS();
        }

        public void Attach(Collidable body)
        {
            this._collidable = body;
        }

        public bool NeedsRotationUpdate => true;

        public void OnRotationUpdate()
        {
            UpdateBox();
        }

        private void UpdateBox()
        {
            var rotMat = this._collidable.transform.rotationMatrix;
            var position = this._collidable.transform.position;

            // Apply rotation matrix to local vertex positions and translate to the correct position
            for (int i = 0; i < localVertices.Length; i++)
            {
                vertices[i] = rotMat * localVertices[i] + position;
            }

            // Calculate axes
            axes[0] = rotMat.GetColumn(0).Normalize();
            axes[1] = rotMat.GetColumn(1).Normalize();
            axes[2] = rotMat.GetColumn(2).Normalize();
        }

        public bool ContainsPoint(in Vector3S worldPoint)
        {
            // Transform the point to local space
            var localPoint = _collidable.transform.ToLocalPoint(worldPoint);

            localPoint.x.AbsInPlace();
            localPoint.y.AbsInPlace();
            localPoint.z.AbsInPlace();
            // Check if the point is within the bounds of the box
            return (localPoint.x) <= halfSize.x &&
                   (localPoint.y) <= halfSize.y &&
                   (localPoint.z) <= halfSize.z;
        }


        public BoundsS CalculateAABB(in Vector3S position, in QuaternionS rotation)
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

        public int Intersects(Collidable other, ref ContactS[] contact)
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

        private static readonly f32 boxInertia = f32.FromFloat(1f / 12f);
        public Matrix3S CalculateInertiaTensor(f32 mass)
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

        public static readonly EdgeS[] BOX_EDGES = new EdgeS[]
        {
            new EdgeS(0, 1), new EdgeS(0, 2), new EdgeS(0, 4),
            new EdgeS(1, 3), new EdgeS(1, 5),
            new EdgeS(2, 3), new EdgeS(2, 6),
            new EdgeS(3, 7),
            new EdgeS(4, 5), new EdgeS(4, 6),
            new EdgeS(5, 7),
            new EdgeS(6, 7),
        };

        public static readonly Vector3S[] BOX_NORMALS = new Vector3S[]
        {
            new Vector3S(1,1,0).Normalize(), new Vector3S(1,0,1).Normalize(), new Vector3S(0,1,1).Normalize(), new Vector3S(1,0,-1).Normalize(),
            new Vector3S(0,1,-1).Normalize(), new Vector3S(1,-1,0).Normalize(), new Vector3S(0,-1,1).Normalize(), new Vector3S(0,-1,-1).Normalize(),
            new Vector3S(-1,1,0).Normalize(), new Vector3S(-1,0,1).Normalize(), new Vector3S(-1,0,-1).Normalize(), new Vector3S(-1,-1,0).Normalize(),
        };

        public void GetAllEdges(ref (Vector3S, Vector3S, Vector3S)[] pairs)
        {
            for (int i = 0; i < BOX_EDGES.Length; i++)
            {
                var e = BOX_EDGES[i];
                pairs[i] = (vertices[e.a], vertices[e.b], BOX_NORMALS[i]);
            }
        }

        public Vector3S Support(Vector3S direction)
        {
            // Transform the direction to local space
            Vector3S localDirection = _collidable.transform.InverseTransformDirection(direction);

            // Calculate the furthest point in the direction of localDirection
            Vector3S localSupportPoint = new Vector3S(
                localDirection.x >= f32.zero ? halfSize.x : -halfSize.x,
                localDirection.y >= f32.zero ? halfSize.y : -halfSize.y,
                localDirection.z >= f32.zero ? halfSize.z : -halfSize.z
            );

            // Transform the support point back to world space
            return _collidable.transform.ToWorldPoint(localSupportPoint);
        }

        public bool RayTest(Vector3S origin, Vector3S direction, f32 maxDistance, out Vector3S point)
        {
            point = Vector3S.zero;

            // Transform the direction to local space and normalize it
            Vector3S localDirection = _collidable.transform.InverseTransformDirection(direction).Normalize();

            // Transform the origin to local space
            Vector3S localOrigin = _collidable.transform.ToLocalPoint(origin);

            // Initialize t values for each face
            f32 tMin = f32.minValue;
            f32 tMax = f32.maxValue;

            // Update tMin and tMax values for an axis
            void UpdateTValues(f32 localDirComponent, f32 localOriginComponent, f32 halfSizeComponent)
            {
                if (localDirComponent != f32.zero)
                {
                    f32 t1 = (halfSizeComponent - localOriginComponent) / localDirComponent;
                    f32 t2 = (-halfSizeComponent - localOriginComponent) / localDirComponent;
                    tMin = MathS.Max(tMin, MathS.Min(t1, t2));
                    tMax = MathS.Min(tMax, MathS.Max(t1, t2));
                }
                else if (localOriginComponent < -halfSizeComponent || localOriginComponent > halfSizeComponent)
                {
                    tMin = f32.maxValue;
                    tMax = f32.minValue;
                }
            }

            // Calculate t values for each axis
            UpdateTValues(localDirection.x, localOrigin.x, halfSize.x);
            UpdateTValues(localDirection.y, localOrigin.y, halfSize.y);
            UpdateTValues(localDirection.z, localOrigin.z, halfSize.z);

            // If tMax < tMin or tMin is greater than maxDistance, no intersection exists
            if (tMax < tMin || tMin > maxDistance || tMax < f32.zero)
            {
                return false;
            }

            // Calculate the intersection point in local space using the valid t value
            f32 t = tMin >= f32.zero ? tMin : tMax; // Use tMin if it's positive, otherwise use tMax
            if (t > maxDistance)
            {
                return false;
            }

            Vector3S localIntersectionPoint = localOrigin + localDirection * t;

            // Transform the point back to world space
            point = _collidable.transform.ToWorldPoint(localIntersectionPoint);

            return true;
        }

        public Vector3S GetIntersectionPointFromLocalCenter(Vector3S worldDirection)
        {
            // Transform the direction to local space
            Vector3S localDirection = _collidable.transform.InverseTransformDirection(worldDirection);

            // Normalize the direction vector
            localDirection.Normalize();

            // Origin is the center of the box in local coordinates (0,0,0)
            Vector3S localOrigin = Vector3S.zero;

            // Initialize t values for each face
            f32 tMin = f32.minValue;
            f32 tMax = f32.maxValue;

            // Calculate t values for each axis
            void UpdateTValues(f32 localDirComponent, f32 localOriginComponent, f32 halfSizeComponent)
            {
                if (localDirComponent != f32.zero)
                {
                    f32 t1 = (halfSizeComponent - localOriginComponent) / localDirComponent;
                    f32 t2 = (-halfSizeComponent - localOriginComponent) / localDirComponent;
                    tMin = MathS.Max(tMin, MathS.Min(t1, t2));
                    tMax = MathS.Min(tMax, MathS.Max(t1, t2));
                }
            }

            UpdateTValues(localDirection.x, localOrigin.x, halfSize.x);
            UpdateTValues(localDirection.y, localOrigin.y, halfSize.y);
            UpdateTValues(localDirection.z, localOrigin.z, halfSize.z);

            // If tMax < tMin, no intersection exists
            if (tMax < tMin)
            {
                return new Vector3S(); // No intersection, return a default value
            }

            // Calculate the intersection point in local space using tMin (since we want the first intersection point)
            Vector3S localIntersectionPoint = localOrigin + localDirection * tMax;

            // Transform the point back to world space
            return _collidable.transform.ToWorldPoint(localIntersectionPoint);
        }
    }
}
