using stupid.Maths;
using System;

namespace stupid.Colliders
{
    public struct BoxColliderS : IShape
    {
        public readonly Vector3S size, halfSize;
        public Vector3S[] vertices, axes;
        private Collidable _collidable;
        public Collidable collidable => _collidable;

        private BoundsS _bounds;
        public BoundsS bounds => _bounds;

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;

            // Initialize vertices array
            this.vertices = new Vector3S[8];

            // Initialize axes array
            this.axes = new Vector3S[3];

            // Initialize other fields
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

            // Define the half-size along each axis
            var right = rotMat.GetColumn(0) * halfSize.x;
            var up = rotMat.GetColumn(1) * halfSize.y;
            var forward = rotMat.GetColumn(2) * halfSize.z;

            // Update vertex positions based on rotation and translation
            vertices[0] = position + right + up + forward;
            vertices[1] = position + right + up - forward;
            vertices[2] = position + right - up + forward;
            vertices[3] = position + right - up - forward;
            vertices[4] = position - right + up + forward;
            vertices[5] = position - right + up - forward;
            vertices[6] = position - right - up + forward;
            vertices[7] = position - right - up - forward;

            // Update axes
            axes[0] = right.Normalize();  // Local X axis
            axes[1] = up.Normalize();     // Local Y axis
            axes[2] = forward.Normalize(); // Local Z axis
        }

        public bool ContainsPoint(in Vector3S worldPoint)
        {
            var localPoint = _collidable.transform.ToLocalPoint(worldPoint);
            localPoint.x.AbsInPlace();
            localPoint.y.AbsInPlace();
            localPoint.z.AbsInPlace();
            return localPoint.x <= halfSize.x &&
                   localPoint.y <= halfSize.y &&
                   localPoint.z <= halfSize.z;
        }

        public BoundsS CalculateAABB(in Vector3S position, in QuaternionS rotation)
        {
            var rotationMatrix = Matrix3S.Rotate(rotation);

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

        public Matrix3S CalculateInertiaTensor(f32 mass)
        {
            var h = size.x;
            var d = size.y;
            var w = size.z;
            var inertiaX = f32.FromFloat(1f / 12f) * mass * (d * d + w * w);
            var inertiaY = f32.FromFloat(1f / 12f) * mass * (h * h + w * w);
            var inertiaZ = f32.FromFloat(1f / 12f) * mass * (h * h + d * d);

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

        public Vector3S Support(Vector3S direction)
        {
            Vector3S localDirection = _collidable.transform.InverseTransformDirection(direction);

            Vector3S localSupportPoint = new Vector3S(
                localDirection.x >= f32.zero ? halfSize.x : -halfSize.x,
                localDirection.y >= f32.zero ? halfSize.y : -halfSize.y,
                localDirection.z >= f32.zero ? halfSize.z : -halfSize.z
            );

            return _collidable.transform.ToWorldPoint(localSupportPoint);
        }

        public bool RayTest(Vector3S origin, Vector3S direction, f32 maxDistance, out Vector3S point)
        {
            point = Vector3S.zero;

            Vector3S localDirection = _collidable.transform.InverseTransformDirection(direction).Normalize();
            Vector3S localOrigin = _collidable.transform.ToLocalPoint(origin);

            f32 tMin = f32.minValue;
            f32 tMax = f32.maxValue;

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

            UpdateTValues(localDirection.x, localOrigin.x, halfSize.x);
            UpdateTValues(localDirection.y, localOrigin.y, halfSize.y);
            UpdateTValues(localDirection.z, localOrigin.z, halfSize.z);

            if (tMax < tMin || tMin > maxDistance || tMax < f32.zero)
            {
                return false;
            }

            f32 t = tMin >= f32.zero ? tMin : tMax;
            if (t > maxDistance)
            {
                return false;
            }

            Vector3S localIntersectionPoint = localOrigin + localDirection * t;
            point = _collidable.transform.ToWorldPoint(localIntersectionPoint);

            return true;
        }
    }
}