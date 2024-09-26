using stupid.Constraints;
using stupid.Maths;
using System.Runtime.CompilerServices;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        //EVERYTHING HERE MUST FOLLOW THE convention: Point on A, NORMAL TOWARDS B

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int SphereVSphere(in SphereColliderS a, in SphereColliderS b, ref ContactData[] contacts)
        {
            var bPos = a.GetCollidable.transform.ToLocalPoint(b.GetCollidable.transform.position);

            var localNormal = bPos.NormalizeWithMagnitude(out var distance);
            var combinedRadius = a.radius + b.radius;

            if (distance > combinedRadius)
            {
                return 0;
            }

            var localPoint = localNormal * a.radius;
            f32 penetrationDepth = distance - combinedRadius;

            var worldPoint = a.GetCollidable.transform.ToWorldPoint(localPoint);
            var worldNormal = a.GetCollidable.transform.TransformDirection(localNormal);

            contacts[0] = new ContactData(worldPoint, worldNormal, penetrationDepth, 0);
            return 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int TestBoxVsSphere(in BoxColliderS box, in SphereColliderS sphere, out ContactData contact, bool flip = false)
        {
            contact = new ContactData();

            var boxTrans = box.GetCollidable.transform;
            var sphereTrans = sphere.GetCollidable.transform;

            // Transform the sphere center into the box's local space
            var localSpherePos = boxTrans.ToLocalPoint(sphereTrans.position);
            var halfSize = box.halfSize;

            Vector3S pointOnBox;
            pointOnBox.x = MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x);
            pointOnBox.y = MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y);
            pointOnBox.z = MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z);

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

            contact = new ContactData(worldPoint, worldNormal, -penetrationDepth, 0);
            return 1;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int BoxVsSphere(in BoxColliderS box, in SphereColliderS sphere, ref ContactData[] contacts)
        {
            var test = TestBoxVsSphere(box, sphere, out var contact, false);
            if (test == 1)
            {
                contacts[0] = contact;
                return 1;
            }

            return 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int SphereVsBox(in SphereColliderS sphere, in BoxColliderS box, ref ContactData[] contacts)
        {
            var test = TestBoxVsSphere(box, sphere, out var contact, true);
            if (test == 1)
            {
                contacts[0] = contact;
                return 1;
            }

            return 0;
        }


    }
}
