using stupid.Maths;
using System.Collections.Generic;

namespace stupid.Colliders
{
    public static partial class CollisionMath
    {
        public static int BoxVsBox(BoxColliderS a, BoxColliderS b, ref ContactS contact)
        {
            Vector3S relativePosition = b.collidable.transform.position - a.collidable.transform.position;

            f32 minOverlap = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;

            // Check for overlaps on the primary axes of both boxes
            if (!CheckOverlapOnAxes(relativePosition, a.axes, a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxes(relativePosition, b.axes, a, b, ref minOverlap, ref minAxis)) return 0;

            // Check for overlaps on the cross product of axes pairs
            if (!CheckOverlapOnCrossAxes(relativePosition, a.axes, b.axes, a, b, ref minOverlap, ref minAxis)) return 0;

            Vector3S normal = minAxis.Normalize();

            // Flip the normal if it's pointing in the wrong direction
            if (Vector3S.Dot(normal, relativePosition) > f32.zero)
            {
                normal = -normal;
            }

            Vector3S contactPoint = FindContactPoint(a, b);
            f32 penetrationDepth = minOverlap;

            if (contactPoint != Vector3S.zero)
            {
                contact.point = contactPoint;
                contact.normal = normal;
                contact.penetrationDepth = penetrationDepth;
                return 1;
            }

            return 0;
        }

        private static bool CheckOverlapOnAxes(Vector3S relativePosition, Vector3S[] axes, BoxColliderS a, BoxColliderS b, ref f32 minOverlap, ref Vector3S minAxis)
        {
            foreach (var axis in axes)
            {
                if (!CheckOverlapOnAxis(relativePosition, axis, a, b, ref minOverlap, ref minAxis)) return false;
            }
            return true;
        }

        private static bool CheckOverlapOnCrossAxes(Vector3S relativePosition, Vector3S[] axesA, Vector3S[] axesB, BoxColliderS a, BoxColliderS b, ref f32 minOverlap, ref Vector3S minAxis)
        {
            foreach (var axisA in axesA)
            {
                foreach (var axisB in axesB)
                {
                    if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(axisA, axisB), a, b, ref minOverlap, ref minAxis)) return false;
                }
            }
            return true;
        }

        private static bool CheckOverlapOnAxis(Vector3S relativePosition, Vector3S axis, BoxColliderS a, BoxColliderS b, ref f32 minOverlap, ref Vector3S minAxis)
        {
            if (axis.sqrMagnitude <= f32.epsilon) return true; // Skip zero-length axes

            if (!OverlapOnAxis(relativePosition, axis, a, b, out f32 overlap))
            {
                return false; // No overlap on this axis, no collision
            }

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                minAxis = axis;
            }

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, BoxColliderS a, BoxColliderS b, out f32 overlap)
        {
            f32 projectionA = ProjectBox(axis, a);
            f32 projectionB = ProjectBox(axis, b);
            f32 distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            overlap = projectionA + projectionB - distance;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S axis, BoxColliderS box)
        {
            Vector3S halfSize = box.halfSize;
            var rotMat = box.collidable.transform.rotationMatrix;
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(2), axis));
        }

        private static Vector3S FindContactPoint(BoxColliderS a, BoxColliderS b)
        {
            _contactPoints.Clear();

            AddContactPointsIfInsideOBB(a.vertices, b.collidable.transform.position, b.halfSize, b.collidable.transform.rotationMatrix);
            AddContactPointsIfInsideOBB(b.vertices, a.collidable.transform.position, a.halfSize, a.collidable.transform.rotationMatrix);

            if (_contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            Vector3S sum = Vector3S.zero;
            for (int i = 0; i < _contactPoints.Count; i++)
            {
                sum += _contactPoints[i];
            }

            return sum / (f32)_contactPoints.Count;
        }

        private static void AddContactPointsIfInsideOBB(Vector3S[] vertices, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            foreach (var vertex in vertices)
            {
                if (IsPointInsideOBB(vertex, position, halfSize, rotation))
                {
                    _contactPoints.Add(vertex);
                }
            }
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }
    }
}
