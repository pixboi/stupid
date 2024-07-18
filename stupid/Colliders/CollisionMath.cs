using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class CollisionMath
    {
        private static Vector3S[] _axes = new Vector3S[15];
        private static List<Vector3S> _contactPoints = new List<Vector3S>();

        public static int SphereVSphere(Vector3S positionA, Vector3S positionB, f32 radA, f32 radB, ref ContactS[] contact)
        {
            f32 squaredDistance = Vector3S.DistanceSquared(positionA, positionB);
            f32 combinedRadius = radA + radB;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance > squaredCombinedRadius)
            {
                return -1; // No intersection
            }

            Vector3S direction = (positionA - positionB).NormalizeWithMagnitude(out f32 distance);
            Vector3S point = positionB + direction * radB;
            Vector3S normal = direction;
            f32 penetrationDepth = combinedRadius - distance;

            contact[0] = new ContactS(point, normal, penetrationDepth);
            return 1;
        }

        public static int BoxVsSphere(BoxColliderS box, SphereColliderS sphere, ref ContactS[] contact)
        {
            var boxTrans = box.attachedCollidable.transform;
            var sphereTrans = sphere.attachedCollidable.transform;

            Matrix3S inverseBoxRotation = boxTrans.rotationMatrix.Transpose();
            Vector3S localSpherePosition = inverseBoxRotation * (sphereTrans.position - boxTrans.position);
            Vector3S halfSize = box.halfSize;

            Vector3S closestPoint = new Vector3S(
                MathS.Clamp(localSpherePosition.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePosition.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePosition.z, -halfSize.z, halfSize.z)
            );

            Vector3S distanceVector = localSpherePosition - closestPoint;
            f32 distanceSquared = distanceVector.SqrMagnitude;

            if (distanceSquared > sphere.radius * sphere.radius)
            {
                return 0; // No contact
            }

            f32 distance = MathS.Sqrt(distanceSquared);
            Vector3S normal = distance > f32.epsilon ? distanceVector / distance : Vector3S.one;

            Vector3S point = boxTrans.position + (boxTrans.rotationMatrix * closestPoint);
            Vector3S worldNormal = boxTrans.rotationMatrix * normal;
            f32 penetrationDepth = sphere.radius - distance;

            contact[0] = new ContactS(point, worldNormal, penetrationDepth);
            return 1; // One contact point
        }

        public static int BoxVsBox(BoxColliderS a, BoxColliderS b, ref ContactS[] contact)
        {
            Vector3S relativePosition = b.attachedCollidable.transform.position - a.attachedCollidable.transform.position;

            // Combine the axes from both boxes
            _axes[0] = a.axes[0];
            _axes[1] = a.axes[1];
            _axes[2] = a.axes[2];
            _axes[3] = b.axes[0];
            _axes[4] = b.axes[1];
            _axes[5] = b.axes[2];

            // Cross product of each axis pair
            int axisCount = 6;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3S cross = Vector3S.Cross(_axes[i], _axes[3 + j]);
                    if (cross.SqrMagnitude > f32.epsilon)
                    {
                        _axes[axisCount++] = cross.Normalize();
                    }
                }
            }

            f32 minOverlap = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;

            for (int i = 0; i < axisCount; i++)
            {
                Vector3S axis = _axes[i];
                if (!OverlapOnAxis(relativePosition, axis, a, b, out f32 overlap))
                {
                    return -1; // No overlap on this axis, no collision
                }

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minAxis = axis;
                }
            }

            Vector3S normal = minAxis;
            normal.Normalize();

            // Flip the normal if it's pointing in the wrong direction
            if (Vector3S.Dot(normal, relativePosition) > f32.zero)
            {
                normal = -normal;
            }

            Vector3S contactPoint = FindContactPoint(a, b);
            f32 penetrationDepth = minOverlap;

            if (_contactPoints.Count > 0)
            {
                contact[0] = new ContactS(contactPoint, normal, penetrationDepth);
                return 1;
            }

            return 0;
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
            var rotMat = box.attachedCollidable.transform.rotationMatrix;
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(2), axis));
        }

        private static Vector3S FindContactPoint(BoxColliderS a, BoxColliderS b)
        {
            _contactPoints.Clear();

            foreach (var vertexA in a.vertices)
            {
                if (IsPointInsideOBB(vertexA, b.attachedCollidable.transform.position, b.halfSize, b.attachedCollidable.transform.rotationMatrix))
                {
                    _contactPoints.Add(vertexA);
                }
            }

            foreach (var vertexB in b.vertices)
            {
                if (IsPointInsideOBB(vertexB, a.attachedCollidable.transform.position, a.halfSize, a.attachedCollidable.transform.rotationMatrix))
                {
                    _contactPoints.Add(vertexB);
                }
            }

            if (_contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            Vector3S avg = Vector3S.zero;
            foreach (var point in _contactPoints)
            {
                avg += point;
            }

            return avg / (f32)_contactPoints.Count;
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }
    }
}
