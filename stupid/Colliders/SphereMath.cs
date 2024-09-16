using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        //EVERYTHING HERE MUST FOLLOW THE convention: Point on A, NORMAL TOWARDS B
        public static int SphereVSphere(in SphereColliderS a, in SphereColliderS b, ref ContactS[] contacts)
        {
            var bPos = a.collidable.transform.ToLocalPoint(b.collidable.transform.position);

            var localNormal = bPos.NormalizeWithMagnitude(out var distance);
            var combinedRadius = a.radius + b.radius;

            if (distance > combinedRadius)
            {
                return 0;
            }

            var localPoint = localNormal * a.radius;
            f32 penetrationDepth = distance - combinedRadius;

            var worldPoint = a.collidable.transform.ToWorldPoint(localPoint);
            var worldNormal = a.collidable.transform.TransformDirection(localNormal);

            contacts[0] = new ContactS(worldPoint, worldNormal, penetrationDepth, a.collidable, b.collidable, 0);
            return 1;
        }


        static int TestBoxVsSphere(in Collidable a, in Collidable b, in BoxColliderS box, in SphereColliderS sphere, out ContactS contact, bool flip = false)
        {
            contact = new ContactS();

            var boxTrans = box.collidable.transform;
            var sphereTrans = sphere.collidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.ToLocalPoint(sphereTrans.position);
            var halfSize = box.halfSize;

            // Check if the sphere's center is inside the box
            if (MathS.Abs(localSpherePos.x) <= halfSize.x &&
                MathS.Abs(localSpherePos.y) <= halfSize.y &&
                MathS.Abs(localSpherePos.z) <= halfSize.z)
            {
                // Find the nearest face of the box in each dimension
                var dx = halfSize.x - MathS.Abs(localSpherePos.x);
                var dy = halfSize.y - MathS.Abs(localSpherePos.y);
                var dz = halfSize.z - MathS.Abs(localSpherePos.z);

                // Determine the minimum penetration axis
                var minPenetration = MathS.Min(dx, MathS.Min(dy, dz));

                Vector3S worldNormal;

                var right = box.rightAxis;
                var up = box.upAxis;
                var forward = box.forwardAxis;

                if (minPenetration == dx)
                {
                    worldNormal = localSpherePos.x > f32.zero ? right : -right;
                }
                else if (minPenetration == dy)
                {
                    worldNormal = localSpherePos.y > f32.zero ? up : -up;
                }
                else
                {
                    worldNormal = localSpherePos.z > f32.zero ? forward : -forward;
                }

                worldNormal.NormalizeInPlace();

                var penetrationDepth = (sphere.radius * worldNormal).Magnitude();

                // Ensure the normal follows the convention: Point on A, NORMAL TOWARDS B
                if (flip) worldNormal = -worldNormal;

                contact = new ContactS(sphereTrans.position, worldNormal, penetrationDepth, a, b, 0);
                return 1;
            }
            else
            {
                // Find the closest point on the box to the sphere center (outside the box)
                var pointOnBox = new Vector3S(
                    MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                    MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                    MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
                );

                // Calculate the vector from the closest point to the sphere center
                var localNormal = (localSpherePos - pointOnBox).NormalizeWithMagnitude(out var distance);

                // Early exit if there's no intersection
                if (distance > sphere.radius)
                {
                    return 0;
                }

                // Transform the closest point and normal back to world space
                var worldPoint = boxTrans.ToWorldPoint(pointOnBox);
                var worldNormal = boxTrans.TransformDirection(localNormal);
                var penetrationDepth = sphere.radius - distance;

                // Ensure the normal follows the convention: Point on A, NORMAL TOWARDS B
                if (flip) worldNormal = -worldNormal;

                contact = new ContactS(worldPoint, worldNormal, -penetrationDepth, a, b, 0);
                return 1;
            }
        }



        public static int BoxVsSphere(in BoxColliderS box, in SphereColliderS sphere, ref ContactS[] contacts)
        {
            var test = TestBoxVsSphere(box.collidable, sphere.collidable, box, sphere, out var contact, false);
            if (test == 1)
            {
                contacts[0] = contact;
                return 1;
            }

            return 0;
        }

        public static int SphereVsBox(in SphereColliderS sphere, in BoxColliderS box, ref ContactS[] contacts)
        {
            var test = TestBoxVsSphere(sphere.collidable, box.collidable, box, sphere, out var contact, true);
            if (test == 1)
            {
                contacts[0] = contact;
                return 1;
            }

            return 0;
        }


    }
}
