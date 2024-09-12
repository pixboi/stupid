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
            var squaredDistance = bPos.sqrMagnitude;
            var combinedRadius = a.radius + b.radius;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance > squaredCombinedRadius)
            {
                return 0;
            }

            var localNormal = bPos.Normalize();
            var localPoint = localNormal * a.radius;
            var distance = bPos.Magnitude();

            f32 penetrationDepth = distance - combinedRadius;

            var worldPoint = a.collidable.transform.ToWorldPoint(localPoint);
            var worldNormal = a.collidable.transform.TransformDirection(localNormal);

            contacts[0] = new ContactS(worldPoint, worldNormal, penetrationDepth, a.collidable, b.collidable, 0);
            return 1;

            /*
            Vector3S aPos = a.collidable.transform.position;
            Vector3S bPos = b.collidable.transform.position;

            f32 squaredDistance = Vector3S.DistanceSquared(aPos, bPos);
            f32 combinedRadius = a.radius + b.radius;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            //Easy speculative would be just to fatten the AABB and fatten this check
            if (squaredDistance > squaredCombinedRadius)
            {
                return 0; // No intersection
            }

            //Towards B
            Vector3S normal = (bPos - aPos).NormalizeWithMagnitude(out f32 distance);
            Vector3S point = aPos + normal * a.radius;
            f32 penetrationDepth = distance - combinedRadius;

            contacts[0] = new ContactS(point, normal, penetrationDepth, a.collidable, b.collidable, 0);
            return 1;
            */
        }


        static int TestBoxVsSphere(in Collidable a, in Collidable b, in BoxColliderS box, in SphereColliderS sphere, out ContactS contact, bool flip = false)
        {
            contact = new ContactS();

            var boxTrans = box.collidable.transform;
            var sphereTrans = sphere.collidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.ToLocalPoint(sphereTrans.position);
            var halfSize = box.halfSize;

            // Find the closest point on the box to the sphere center
            var localPoint = new Vector3S(
                MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
            );

            // Calculate the vector from the closest point to the sphere center
            var localNormal = (localSpherePos - localPoint).NormalizeWithMagnitude(out var distance);

            // Early exit if there's no intersection
            if (distance > sphere.radius)
                return 0;

            // Transform the closest point and normal back to world space
            var worldPoint = boxTrans.ToWorldPoint(localPoint);
            var worldNormal = boxTrans.TransformDirection(localNormal);
            var penetrationDepth = sphere.radius - distance;

            // Ensure the normal follows the convention: Point on A, NORMAL TOWARDS B
            if (flip) worldNormal = -worldNormal;

            contact = new ContactS(worldPoint, worldNormal, -penetrationDepth, a, b, 0);
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
