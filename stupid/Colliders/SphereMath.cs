using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        public static int SphereVSphere(in SphereColliderS a, in SphereColliderS b, ref ContactS[] contacts)
        {
            Vector3S positionA = a.collidable.transform.position;
            Vector3S positionB = b.collidable.transform.position;

            f32 squaredDistance = Vector3S.DistanceSquared(positionA, positionB);
            f32 combinedRadius = a.radius + b.radius;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance > squaredCombinedRadius)
            {
                return 0; // No intersection
            }

            Vector3S direction = (positionA - positionB).NormalizeWithMagnitude(out f32 distance);
            Vector3S point = positionB + direction * b.radius;
            Vector3S normal = direction;
            f32 penetrationDepth = distance - combinedRadius;

            contacts[0] = new ContactS(point, normal, penetrationDepth, a.collidable, b.collidable);
            return 1;
        }

        public static int BoxVsSphere(in BoxColliderS box, in SphereColliderS sphere, ref ContactS[] contacts)
        {
            var boxTrans = box.collidable.transform;
            var sphereTrans = sphere.collidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.rotationMatrix.Transpose() * (sphereTrans.position - boxTrans.position);
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

            // Calculate the distance and the normal vector pointing from the closest point to the sphere center
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

            // Calculate penetration depth
            var penetrationDepth = distance - sphere.radius;

            contacts[0] = new ContactS(worldClosestPoint, -worldNormal, penetrationDepth, box.collidable, sphere.collidable);
            return 1;
        }



        public static int SphereVsBox(in SphereColliderS sphere, in BoxColliderS box, ref ContactS[] contacts)
        {
            var boxTrans = box.collidable.transform;
            var sphereTrans = sphere.collidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.rotationMatrix.Transpose() * (sphereTrans.position - boxTrans.position);
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

            // Calculate the distance and the normal vector pointing from the closest point to the sphere center
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

            // Calculate penetration depth
            var penetrationDepth = distance - sphere.radius;

            contacts[0] = new ContactS(worldClosestPoint, worldNormal, penetrationDepth, sphere.collidable, box.collidable);
            return 1;
        }


    }
}
