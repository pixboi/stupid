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
            Vector3S aPos = a.collidable.transform.position;
            Vector3S bPos = b.collidable.transform.position;

            f32 squaredDistance = Vector3S.DistanceSquared(aPos, bPos);
            f32 combinedRadius = a.radius + b.radius;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance > squaredCombinedRadius)
            {
                return 0; // No intersection
            }

            //Towards B
            Vector3S normal = (bPos - aPos).NormalizeWithMagnitude(out f32 distance);
            Vector3S point = aPos + normal * a.radius;
            f32 penetrationDepth = distance - combinedRadius;

            contacts[0] = new ContactS(point, normal, penetrationDepth, a.collidable, b.collidable);
            return 1;
        }


        static int TestBoxVsSphere(in Collidable a, in Collidable b, in BoxColliderS box, in SphereColliderS sphere, out ContactS contact, bool flip = false)
        {
            contact = new ContactS();

            var boxTrans = box.collidable.transform;
            var sphereTrans = sphere.collidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.rotationMatrixTransposed * (sphereTrans.position - boxTrans.position);
            var halfSize = box.halfSize;

            // Find the closest point on the box to the sphere center
            var closestPoint = new Vector3S(
                MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
            );

            // Calculate the vector from the closest point to the sphere center
            var distanceVector = localSpherePos - closestPoint;
            var distanceSquared = distanceVector.sqrMagnitude;
            var radiusSquared = sphere.radius * sphere.radius;

            // Early exit if there's no intersection
            if (distanceSquared > radiusSquared)
                return 0;

            // Calculate the distance and the normal vector
            f32 distance = MathS.Sqrt(distanceSquared);
            Vector3S normal;

            if (distance > f32.epsilon)
            {
                normal = distanceVector / distance;
            }
            else
            {
                // If the distance is zero or very small, default to an arbitrary normal
                normal = new Vector3S(1, 0, 0); // Or any axis-aligned vector
            }

            // Transform the closest point and normal back to world space
            var worldClosestPoint = boxTrans.position + (boxTrans.rotationMatrix * closestPoint);

            var worldNormal = (boxTrans.rotationMatrix * normal).Normalize();
            //This makes boxes slippy
            // worldClosestPoint = sphereTrans.position + sphere.radius * worldNormal;

            var penetrationDepth = sphere.radius - distance;

            // Ensure the normal follows the convention: Point on A, NORMAL TOWARDS B
            if (flip)
            {
                worldNormal = -worldNormal;
            }

            contact = new ContactS(worldClosestPoint, worldNormal, -penetrationDepth, a, b);
            return 1;
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
