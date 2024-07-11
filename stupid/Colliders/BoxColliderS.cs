﻿using stupid.Maths;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }
        public Vector3S halfSize { get; private set; }
        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
        }

        public override BoundsS CalculateAABB(Vector3S position)
        {
            // Get the rotation matrix from the quaternion
            var rotationMatrix = Matrix3S.Rotate(attachedCollidable.transform.rotation);

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
                return BoxHelpers.IntersectBox(
                    this.attachedCollidable.transform.position,
                    this.attachedCollidable.transform.rotation,
                    this.size,
                    other.transform.position,
                    other.transform.rotation,
                    otherBox.size,
                    out contact);
            }

            if (other.collider is SphereColliderS otherSphere)
            {
                return BoxHelpers.IntersectBoxSphere(
                    this.attachedCollidable.transform.position,
                    this.attachedCollidable.transform.rotation,
                    this.size,
                    other.transform.position,
                    otherSphere.radius,
                    out contact);
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
