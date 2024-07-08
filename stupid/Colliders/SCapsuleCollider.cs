using stupid.Maths;

namespace stupid.Colliders
{
    public class SCapsuleCollider : BaseCollider
    {
        public f32 Radius { get; private set; }
        public f32 Height { get; private set; }

        public SCapsuleCollider(f32 radius, f32 height)
        {
            Radius = radius;
            Height = height;
        }

        public override SBounds CalculateBounds(Vector3S position)
        {
            var size = new Vector3S(Radius, Radius + Height * f32.half, Radius);
            _bounds = new SBounds(position - size, position + size);
            return _bounds;
        }

        public override SBounds GetBounds() => _bounds;

        public override bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
        {
            contact = new Contact();

            if (other is SSphereCollider otherSphere)
            {
                return IntersectsSphere(positionA, positionB, otherSphere, out contact);
            }
            else if (other is SBoxCollider otherBox)
            {
                return otherBox.Intersects(positionB, positionA, this, out contact);
            }
            else if (other is SCapsuleCollider otherCapsule)
            {
                return IntersectsCapsule(positionA, positionB, otherCapsule, out contact);
            }

            return false;
        }

        private bool IntersectsSphere(Vector3S positionA, Vector3S positionB, SSphereCollider otherSphere, out Contact contact)
        {
            contact = new Contact();

            var capsuleClosestPoint = ClosestPointOnCapsule(positionA, positionB);
            var distanceSquared = Vector3S.DistanceSquared(capsuleClosestPoint, positionB);
            var combinedRadius = Radius + otherSphere.Radius;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            if (distanceSquared > squaredCombinedRadius)
            {
                return false;
            }

            var direction = (positionB - capsuleClosestPoint).NormalizeWithMagnitude(out var distance);

            contact.point = capsuleClosestPoint + direction * Radius;
            contact.normal = direction;
            contact.penetrationDepth = combinedRadius - distance;

            return true;
        }

        private bool IntersectsCapsule(Vector3S positionA, Vector3S positionB, SCapsuleCollider otherCapsule, out Contact contact)
        {
            contact = new Contact();

            var capsuleAClosestPoint = ClosestPointOnCapsule(positionA, positionB);
            var capsuleBClosestPoint = otherCapsule.ClosestPointOnCapsule(positionB, positionA);

            var distanceSquared = Vector3S.DistanceSquared(capsuleAClosestPoint, capsuleBClosestPoint);
            var combinedRadius = Radius + otherCapsule.Radius;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            if (distanceSquared > squaredCombinedRadius)
            {
                return false;
            }

            var direction = (capsuleBClosestPoint - capsuleAClosestPoint).NormalizeWithMagnitude(out var distance);

            contact.point = capsuleAClosestPoint + direction * Radius;
            contact.normal = direction;
            contact.penetrationDepth = combinedRadius - distance;

            return true;
        }

        private Vector3S ClosestPointOnCapsule(Vector3S positionA, Vector3S point)
        {
            var halfHeight = Height * f32.half;
            var top = positionA + new Vector3S(f32.zero, halfHeight, f32.zero);
            var bottom = positionA - new Vector3S(f32.zero, halfHeight, f32.zero);

            var topToPoint = point - top;
            var bottomToPoint = point - bottom;

            if (Vector3S.Dot(topToPoint, bottomToPoint) <= f32.zero)
            {
                return point;
            }
            else if (topToPoint.SqrMagnitude < bottomToPoint.SqrMagnitude)
            {
                return top;
            }
            else
            {
                return bottom;
            }
        }

        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            throw new System.NotImplementedException();
        }
    }
}
