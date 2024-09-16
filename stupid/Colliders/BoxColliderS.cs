using stupid.Maths;

namespace stupid.Colliders
{
    public struct BoxColliderS : IShape
    {
        // Static array of vertex positions relative to the box center
        private static readonly Vector3S[] predefinedVertices = new Vector3S[8]
        {
        new Vector3S(1, 1, 1),
        new Vector3S(1, 1, -1),
        new Vector3S(1, -1, 1),
        new Vector3S(1, -1, -1),
        new Vector3S(-1, 1, 1),
        new Vector3S(-1, 1, -1),
        new Vector3S(-1, -1, 1),
        new Vector3S(-1, -1, -1),
        };

        public readonly Vector3S halfSize;
        public Vector3S size => this.halfSize * f32.two;

        public readonly Vector3S[] localVertices;
        public Vector3S[] vertices;
        public Vector3S rightAxis;
        public Vector3S upAxis;
        public Vector3S forwardAxis;

        private Collidable _collidable;
        public Collidable collidable => _collidable;

        private BoundsS _bounds;
        public BoundsS bounds => _bounds;

        public BoxColliderS(Vector3S size)
        {
            this.halfSize = size * f32.half;

            // Initialize local vertices array
            this.localVertices = new Vector3S[8];

            // Initialize world vertices array
            this.vertices = new Vector3S[8];

            // Initialize other fields
            this._collidable = null;
            this._bounds = new BoundsS();

            // Define the local vertices (fixed, relative to the box center)
            this.localVertices[0] = new Vector3S(halfSize.x, halfSize.y, halfSize.z);
            this.localVertices[1] = new Vector3S(halfSize.x, halfSize.y, -halfSize.z);
            this.localVertices[2] = new Vector3S(halfSize.x, -halfSize.y, halfSize.z);
            this.localVertices[3] = new Vector3S(halfSize.x, -halfSize.y, -halfSize.z);
            this.localVertices[4] = new Vector3S(-halfSize.x, halfSize.y, halfSize.z);
            this.localVertices[5] = new Vector3S(-halfSize.x, halfSize.y, -halfSize.z);
            this.localVertices[6] = new Vector3S(-halfSize.x, -halfSize.y, halfSize.z);
            this.localVertices[7] = new Vector3S(-halfSize.x, -halfSize.y, -halfSize.z);

            this.rightAxis = Vector3S.right;
            this.upAxis = Vector3S.up;
            this.forwardAxis = Vector3S.forward;
        }

        void UpdateAxis()
        {
            this.rightAxis = this._collidable.transform.rotationMatrix.GetColumn(0).Normalize();
            this.upAxis = this._collidable.transform.rotationMatrix.GetColumn(1).Normalize();
            this.forwardAxis = this._collidable.transform.rotationMatrix.GetColumn(2).Normalize();
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
            // Update vertex positions based on rotation and translation
            for (int i = 0; i < 8; i++)
            {
                vertices[i] = this._collidable.transform.ToWorldPoint(localVertices[i]);
            }

            UpdateAxis();
        }


        public bool ContainsPoint(in Vector3S worldPoint)
        {
            var absLocal = Vector3S.Abs(_collidable.transform.ToLocalPoint(worldPoint));
            var fat = f32.epsilon;

            return absLocal.x <= halfSize.x &&
                   absLocal.y <= halfSize.y &&
                   absLocal.z <= halfSize.z;
        }

        public BoundsS CalculateAABB(in TransformS t)
        {
            Vector3S rotatedHalfSize = new Vector3S(
    MathS.Abs(t.rotationMatrix.m00) * halfSize.x + MathS.Abs(t.rotationMatrix.m01) * halfSize.y + MathS.Abs(t.rotationMatrix.m02) * halfSize.z,
    MathS.Abs(t.rotationMatrix.m10) * halfSize.x + MathS.Abs(t.rotationMatrix.m11) * halfSize.y + MathS.Abs(t.rotationMatrix.m12) * halfSize.z,
    MathS.Abs(t.rotationMatrix.m20) * halfSize.x + MathS.Abs(t.rotationMatrix.m21) * halfSize.y + MathS.Abs(t.rotationMatrix.m22) * halfSize.z
);

            var min = t.position - rotatedHalfSize;
            var max = t.position + rotatedHalfSize;
            _bounds = new BoundsS(min, max);
            return _bounds;
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


        static f32 boxConst = (f32)(1f / 12f);
        public Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // Precompute constants and squares to minimize redundant calculations
            var massConst = boxConst * mass;

            var h2 = size.x * size.x; // h squared
            var d2 = size.y * size.y; // d squared
            var w2 = size.z * size.z; // w squared

            // Calculate inertia tensor components
            var inertiaX = massConst * (d2 + w2);
            var inertiaY = massConst * (h2 + w2);
            var inertiaZ = massConst * (h2 + d2);

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

        public bool RayTest(in Vector3S origin, in Vector3S direction, in f32 maxDistance, out Vector3S point, out f32 distance)
        {
            point = Vector3S.zero;
            distance = f32.maxValue; // Start with max value

            Vector3S localDirection = _collidable.transform.InverseTransformDirection(direction).Normalize();
            Vector3S localOrigin = _collidable.transform.ToLocalPoint(origin);

            // Initialize tMin and tMax
            f32 tMin = -f32.maxValue;
            f32 tMax = f32.maxValue;

            // Function to update tMin and tMax based on ray direction and box half-size
            void UpdateTValues(f32 localDirComponent, f32 localOriginComponent, f32 halfSizeComponent)
            {
                if (MathS.Abs(localDirComponent) > f32.epsilon) // Avoid division by zero
                {
                    f32 invDir = f32.one / localDirComponent; // Inverse direction
                    f32 t1 = (halfSizeComponent - localOriginComponent) * invDir;
                    f32 t2 = (-halfSizeComponent - localOriginComponent) * invDir;

                    // Ensure t1 is the minimum and t2 is the maximum
                    if (t1 > t2)
                    {
                        var temp = t1;
                        t1 = t2;
                        t2 = temp;
                    }

                    tMin = MathS.Max(tMin, t1);
                    tMax = MathS.Min(tMax, t2);
                }
                else if (localOriginComponent < -halfSizeComponent || localOriginComponent > halfSizeComponent)
                {
                    // Ray is parallel to the slab and outside the bounds
                    tMin = f32.maxValue;
                    tMax = f32.minValue;
                }
            }

            // Update tMin and tMax for each axis
            UpdateTValues(localDirection.x, localOrigin.x, halfSize.x);
            UpdateTValues(localDirection.y, localOrigin.y, halfSize.y);
            UpdateTValues(localDirection.z, localOrigin.z, halfSize.z);

            // Check if the ray intersects the box
            if (tMax < tMin || tMin > maxDistance || tMax < f32.zero)
            {
                return false;
            }

            // Calculate the closest intersection point within bounds
            f32 t = tMin >= f32.zero ? tMin : tMax;
            if (t > maxDistance || t < f32.zero)
            {
                return false;
            }

            Vector3S localIntersectionPoint = localOrigin + localDirection * t;
            point = _collidable.transform.ToWorldPoint(localIntersectionPoint);
            distance = t; // The distance from the ray's origin to the intersection point

            return true;
        }



    }
}