using stupid.Constraints;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public class BoxColliderS : Shape
    {
        public static readonly EdgeS[] BOX_EDGES = new EdgeS[]
{
            //Aligned on X LOCAL
            new EdgeS(4, 0), new EdgeS(5, 1), new EdgeS(7, 3), new EdgeS(6, 2),
            //Aligned on Y
            new EdgeS(6, 4), new EdgeS(2, 0), new EdgeS(7, 5), new EdgeS(3, 1),
            //Aligned on Z
            new EdgeS(7, 6), new EdgeS(5, 4), new EdgeS(1, 0), new EdgeS(3, 2),
};
        public static readonly EdgeS[] BOX_EDGES_RIGHT = new EdgeS[]
        {
                        //Aligned on X LOCAL
            new EdgeS(4, 0), new EdgeS(5, 1), new EdgeS(7, 3), new EdgeS(6, 2),
        };
        public static readonly EdgeS[] BOX_EDGES_UP = new EdgeS[]
{
            //Aligned on Y
            new EdgeS(6, 4), new EdgeS(2, 0), new EdgeS(7, 5), new EdgeS(3, 1),
};
        public static readonly EdgeS[] BOX_EDGES_FORWARD = new EdgeS[]
{
            //Aligned on Z
            new EdgeS(7, 6), new EdgeS(5, 4), new EdgeS(1, 0), new EdgeS(3, 2),
};
        public static readonly Vector3S[] BOX_NORMALS = new Vector3S[]
{
            //Aligned on X LOCAL
            new Vector3S(0,1,1).Normalize(),new Vector3S(0,1,-1).Normalize(),new Vector3S(0,-1,-1).Normalize(),new Vector3S(0,-1,1).Normalize(),
            //Aligned on Y
            new Vector3S(-1,0,1).Normalize(), new Vector3S(1,0,1).Normalize(), new Vector3S(-1,0,-1).Normalize(), new Vector3S(1,0,-1).Normalize(),
            //Aligned on Z
            new Vector3S(-1,-1,0).Normalize(), new Vector3S(-1,1,0).Normalize(),new Vector3S(1,1,0).Normalize(), new Vector3S(1,-1,0).Normalize(),
};
        public static EdgeS[] GetEdges(int axis)
        {
            if (axis == 0)
            {
                return BOX_EDGES_RIGHT;
            }
            else if (axis == 1)
            {
                return BOX_EDGES_UP;
            }
            else
            {
                return BOX_EDGES_FORWARD;
            }
        }
        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;

            // Initialize local vertices array
            this.localVertices = new Vector3S[8]
            {
                 new Vector3S(halfSize.x, halfSize.y, halfSize.z),
                 new Vector3S(halfSize.x, halfSize.y, -halfSize.z),
                 new Vector3S(halfSize.x, -halfSize.y, halfSize.z),
                 new Vector3S(halfSize.x, -halfSize.y, -halfSize.z),
                 new Vector3S(-halfSize.x, halfSize.y, halfSize.z),
                 new Vector3S(-halfSize.x, halfSize.y, -halfSize.z),
                 new Vector3S(-halfSize.x, -halfSize.y, halfSize.z),
                 new Vector3S(-halfSize.x, -halfSize.y, -halfSize.z)
            };

            // Initialize world vertices array
            this.vertices = new Vector3S[8];

            this.rightAxis = Vector3S.right;
            this.upAxis = Vector3S.up;
            this.forwardAxis = Vector3S.forward;
        }

        //Init
        public readonly Vector3S size;
        public readonly Vector3S halfSize;
        public readonly Vector3S[] localVertices;


        //Runtime
        public Vector3S rightAxis, upAxis, forwardAxis;
        public Vector3S[] vertices;

        public Vector3S GetAxis(int index)
        {
            switch (index)
            {
                case 0: return rightAxis;
                case 1: return upAxis;
                case 2: return forwardAxis;
            }

            throw new System.ArgumentOutOfRangeException(index.ToString());
        }

        public override bool NeedsRotationUpdate => true;
        public override void OnRotationUpdate()
        {
            // Update vertex positions based on rotation and translation
            for (int i = 0; i < 8; i++)
            {
                vertices[i] = this.collidable.transform.ToWorldPoint(localVertices[i]);
            }

            this.rightAxis = this.collidable.transform.rotationMatrix.GetColumn(0).Normalize();
            this.upAxis = this.collidable.transform.rotationMatrix.GetColumn(1).Normalize();
            this.forwardAxis = this.collidable.transform.rotationMatrix.GetColumn(2).Normalize();
        }

        public bool ContainsPoint(in Vector3S worldPoint)
        {
            var absLocal = Vector3S.Abs(collidable.transform.ToLocalPoint(worldPoint));

            var fat = f32.epsilon;

            return absLocal.x <= halfSize.x + fat &&
                   absLocal.y <= halfSize.y + fat &&
                   absLocal.z <= halfSize.z + fat;
        }

        public override BoundsS GetBounds(TransformS t)
        {
            Vector3S rotatedHalfSize = new Vector3S(
    MathS.Abs(t.rotationMatrix.m00) * halfSize.x + MathS.Abs(t.rotationMatrix.m01) * halfSize.y + MathS.Abs(t.rotationMatrix.m02) * halfSize.z,
    MathS.Abs(t.rotationMatrix.m10) * halfSize.x + MathS.Abs(t.rotationMatrix.m11) * halfSize.y + MathS.Abs(t.rotationMatrix.m12) * halfSize.z,
    MathS.Abs(t.rotationMatrix.m20) * halfSize.x + MathS.Abs(t.rotationMatrix.m21) * halfSize.y + MathS.Abs(t.rotationMatrix.m22) * halfSize.z
);
            var min = t.position - rotatedHalfSize;
            var max = t.position + rotatedHalfSize;
            return new BoundsS(min, max);
        }

        public override int Intersects(Collidable other, ref ContactData[] contact)
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
        public override Matrix3S CalculateInertiaTensor(in f32 mass)
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

        public Vector3S CalculateTensorDiagonal(in f32 mass)
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

            return new Vector3S(inertiaX, inertiaY, inertiaZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RaycastBox(Vector3S rayOrigin, Vector3S rayDirection, out Vector3S intersectionPoint, out f32 tMin)
        {
            // Transform the ray to the local space of the box
            Vector3S localOrigin = collidable.transform.ToLocalPoint(rayOrigin);
            Vector3S localDirection = collidable.transform.InverseTransformDirection(rayDirection).Normalize();


            // Initialize intersection parameters
            tMin = f32.minValue;
            f32 tMax = f32.maxValue;

            // Iterate over the three axes of the box
            for (int i = 0; i < 3; i++)
            {
                // Get the ray's direction component for the current axis
                f32 dirComponent = localDirection[i];
                f32 originComponent = localOrigin[i];
                f32 halfSizeComponent = halfSize[i];

                // Check if the ray is parallel to the axis
                if (MathS.Abs(dirComponent) > f32.epsilon)
                {
                    f32 t1 = (-halfSizeComponent - originComponent) / dirComponent;
                    f32 t2 = (halfSizeComponent - originComponent) / dirComponent;

                    if (t1 > t2)
                    {
                        var temp = t1;
                        t1 = t2;
                        t2 = temp;
                    }

                    tMin = MathS.Max(tMin, t1);
                    tMax = MathS.Min(tMax, t2);

                    // If the intervals do not overlap, there is no intersection
                    if (tMax < tMin)
                    {
                        intersectionPoint = Vector3S.zero;
                        return false;
                    }
                }
                else if (originComponent < -halfSizeComponent || originComponent > halfSizeComponent)
                {
                    // Ray is parallel to the axis and outside the box
                    intersectionPoint = Vector3S.zero;
                    return false;
                }
            }

            // Calculate the intersection point in local space
            intersectionPoint = localOrigin + localDirection * tMin;

            // Transform the intersection point back to world space
            intersectionPoint = collidable.transform.ToWorldPoint(intersectionPoint);
            return true;
        }

    }
}