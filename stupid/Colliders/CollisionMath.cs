using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class CollisionMath
    {
        public static int SphereVSphere(Vector3S positionA, Vector3S positionB, f32 radA, f32 radB, ref ContactS[] contact)
        {
            // Calculate the squared distance between the two positions
            var squaredDistance = Vector3S.DistanceSquared(positionA, positionB);

            // Calculate the combined radius of both spheres
            var combinedRadius = radA + radB;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            // If the squared distance is greater than the squared combined radius, there is no intersection
            if (squaredDistance > squaredCombinedRadius)
            {
                return -1;
            }

            // Calculate direction and distance between the spheres
            var direction = (positionA - positionB).NormalizeWithMagnitude(out var distance);

            // Set contact information
            var point = positionB + direction * radB; // Contact point on the surface of B
            var normal = direction; // Normal points from B => A
            var penetrationDepth = combinedRadius - distance;
            var c = new ContactS(point, normal, penetrationDepth);

            contact[0] = c;
            return 1;
        }


        public static int BoxVsSphere(BoxColliderS box, SphereColliderS sphere, ref ContactS[] contact)
        {
            var boxTrans = box.attachedCollidable.transform;
            var sphereTrans = sphere.attachedCollidable.transform;

            // Transform the sphere center into the box's local space
            Matrix3S boxRotationMatrix = boxTrans.rotationMatrix;
            Matrix3S inverseBoxRotation = boxRotationMatrix.Transpose(); // Efficient inverse for rotation matrix
            Vector3S localSpherePosition = inverseBoxRotation * (sphereTrans.position - boxTrans.position);
            Vector3S halfSize = box.halfSize;

            // Clamp the sphere's local position to the box's extents to find the closest point
            Vector3S closestPoint = new Vector3S(
                MathS.Clamp(localSpherePosition.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePosition.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePosition.z, -halfSize.z, halfSize.z)
            );

            // Calculate the distance between the sphere's center and this closest point
            Vector3S distanceVector = localSpherePosition - closestPoint;
            f32 distanceSquared = distanceVector.SqrMagnitude;

            // If the distance is greater than the sphere's radius, there's no intersection
            if (distanceSquared > sphere.radius * sphere.radius)
            {
                return -1;
            }

            // Calculate the actual distance and normalize the distance vector
            f32 distance = MathS.Sqrt(distanceSquared);
            Vector3S normal = distance > f32.epsilon ? distanceVector / distance : Vector3S.zero;

            // Transform the closest point back to world space
            var point = boxTrans.position + (boxRotationMatrix * closestPoint);

            // If the box is dynamic and the sphere is static, flip the normal
            if (box.attachedCollidable.isDynamic && !sphere.attachedCollidable.isDynamic)
            {
                normal = -normal;
            }

            // Transform the normal back to world space
            var worldNormal = boxRotationMatrix * normal;
            var penetrationDepth = sphere.radius - distance;

            contact[0] = new ContactS(point, worldNormal, penetrationDepth);
            return 1;
        }




        static Vector3S[] _axes = new Vector3S[15];
        static List<Vector3S> _contactPoints = new List<Vector3S>();

        public static int BoxVsBox(BoxColliderS a, BoxColliderS b, ref ContactS[] contact)
        {
            Vector3S relativePosition = (b.attachedCollidable.transform.position - a.attachedCollidable.transform.position);

            _axes[0] = a.axes[0];
            _axes[1] = a.axes[1];
            _axes[2] = a.axes[2];
            _axes[3] = b.axes[0];
            _axes[4] = b.axes[1];
            _axes[5] = b.axes[2];

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
            //This comes from the axis calcs
            if (Vector3S.Dot(normal, relativePosition) > f32.zero)
            {
                normal = -normal;
            }

            var bRot = b.attachedCollidable.transform.rotationMatrix;

            Vector3S contactPoint = FindContactPoint(a.attachedCollidable.transform.position, a.halfSize, a.attachedCollidable.transform.rotationMatrix, a.vertices,
                b.attachedCollidable.transform.position, b.halfSize, bRot, b.vertices);

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
            Vector3S halfSize = box.size * f32.half;
            var rotMat = box.attachedCollidable.transform.rotationMatrix;
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(2), axis));
        }

        private static Vector3S FindContactPoint(Vector3S positionA, Vector3S halfSizeA, Matrix3S rotationA, Vector3S[] verticesA,
            Vector3S positionB, Vector3S halfSizeB, Matrix3S rotationB, Vector3S[] verticesB)
        {
            _contactPoints.Clear();

            foreach (var vertexA in verticesA)
            {
                if (IsPointInsideOBB(vertexA, positionB, halfSizeB, rotationB))
                {
                    _contactPoints.Add(vertexA);
                }
            }

            foreach (var vertexB in verticesB)
            {
                if (IsPointInsideOBB(vertexB, positionA, halfSizeA, rotationA))
                {
                    _contactPoints.Add(vertexB);
                }
            }

            if (_contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            var avg = Vector3S.zero;
            foreach (var c in _contactPoints) avg += c;

            return avg / (f32)_contactPoints.Count;
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }
    }
}
