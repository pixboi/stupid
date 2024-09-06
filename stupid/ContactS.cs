﻿using stupid.Maths;
using stupid;
using System.Runtime.CompilerServices;

public struct ContactS
{
    public readonly Vector3S point, normal, tangent1, tangent2, localAnchorA, localAnchorB;
    public readonly f32 normalMass, tangentMass1, tangentMass2, twistMass;
    public readonly f32 penetrationDepth;
    public readonly int featureId;

    public f32 accumulatedImpulse, accumulatedFriction, accumulatedTwist;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b, int featureId = 0)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.featureId = featureId;

        this.localAnchorA = this.point - a.transform.position;
        this.localAnchorB = this.point - b.transform.position;

        this.normalMass = f32.zero;
        this.tangent1 = Vector3S.zero;
        this.tangent2 = Vector3S.zero;
        this.tangentMass1 = f32.zero;
        this.tangentMass2 = f32.zero;
        this.twistMass = f32.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;
        this.accumulatedTwist = f32.zero;

        if (a.isDynamic || b.isDynamic)
        {
            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            this.normalMass = CalculateMassNormal(ab, bb);

            CalculateTangentAndMass(ab, bb, out var t1, out var m1, out var t2, out var m2);
            this.tangent1 = t1;
            this.tangentMass1 = m1;
            this.tangent2 = t2;
            this.tangentMass2 = m2;
            this.twistMass = CalculateTwistMass(ab, bb);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WarmStart(in RigidbodyS a, in Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Reapply the accumulated impulses
        var normalImpulse = (this.normal * this.accumulatedImpulse);
        var tangentImpulse = (this.tangent1 * this.accumulatedFriction);

        Vector3S warmImpulse = normalImpulse + tangentImpulse;
        ApplyImpulse(a, bb, warmImpulse);

        ApplyTwistImpulse(a, bb, this.accumulatedTwist);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateMassNormal(RigidbodyS ab, RigidbodyS bb)
    {
        f32 invMassA = ab.inverseMass;
        f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

        f32 effectiveMass = invMassA + invMassB;

        Vector3S raCrossNormal = Vector3S.Cross(this.localAnchorA, this.normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
        effectiveMass += angularMassA;

        if (bb != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(this.localAnchorB, this.normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += angularMassB;
        }

        return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void CalculateTangentAndMass(in RigidbodyS a, in RigidbodyS b, out Vector3S t1, out f32 m1, out Vector3S t2, out f32 m2)
    {
        // Calculate relative velocity at the contact point
        var contactVelocity = CalculateContactVelocity(a, b);

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;

        // Define a threshold for when the tangential velocity is considered too small
        f32 tangentialVelocityThreshold = f32.epsilon;

        // Use tangential velocity if it is significant
        if (tangentialVelocity.sqrMagnitude > tangentialVelocityThreshold)
        {
            // Tangent 1 aligned with sliding direction (tangential velocity)
            t1 = tangentialVelocity.Normalize();
        }
        else
        {
            // Fallback mechanism: Use last known good tangent direction (persistent tangents between frames)
            // If this is not feasible, fall back to an arbitrary perpendicular direction
            t1 = Vector3S.Cross(Vector3S.right, this.normal).Normalize();  // Try with X-axis
            if (t1.sqrMagnitude < f32.epsilon)
                t1 = Vector3S.Cross(Vector3S.up, this.normal).Normalize(); // Try with Y-axis as fallback
        }

        // Calculate effective mass along the first tangent (t1)
        m1 = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorA, t1), this.localAnchorA), t1);
        if (b != null)
            m1 += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorB, t1), this.localAnchorB), t1);
        m1 = f32.one / m1;  // Invert the mass to get the effective mass

        // Calculate the second tangent (t2) as perpendicular to both the normal and the first tangent (t1)
        t2 = Vector3S.Cross(this.normal, t1).Normalize();

        // Calculate effective mass along the second tangent (t2)
        m2 = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorA, t2), this.localAnchorA), t2);
        if (b != null)
            m2 += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorB, t2), this.localAnchorB), t2);
        m2 = f32.one / m2;  // Invert the mass to get the effective mass
    }



    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateTwistMass(in RigidbodyS a, in RigidbodyS bb)
    {
        // Effective mass for twist friction is based on the moment of inertia along the normal
        var twistMassA = Vector3S.Dot(this.normal, a.tensor.inertiaWorld * this.normal);
        var twistMassB = bb != null ? Vector3S.Dot(this.normal, bb.tensor.inertiaWorld * this.normal) : f32.zero;

        var totalTwistMass = twistMassA + twistMassB;
        return totalTwistMass > f32.zero ? f32.one / totalTwistMass : f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool bias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;
        var baum = f32.zero;

        if (this.penetrationDepth > f32.zero)
        {
            baum = this.penetrationDepth * inverseDt;
        }
        else if (bias)
        {
            var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);
            baum = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var contactVelocity = CalculateContactVelocity(a, bb);
        var vn = Vector3S.Dot(contactVelocity, this.normal);

        var impulse = -this.normalMass * (vn + baum);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = this.normal * impulse;
        ApplyImpulse(a, bb, normalImpulse);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var contactVelocity = CalculateContactVelocity(a, bb);

        var vt = Vector3S.Dot(contactVelocity, this.tangent1);
        var incrementalFriction = -this.tangentMass1 * vt;

        /*
        var vt1 = Vector3S.Dot(contactVelocity, this.tangent2);
        var incrementalFriction1 = -this.tangentMass2 * vt1;
        incrementalFriction += incrementalFriction1;
        */

        var couloumbMax = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        ApplyImpulse(a, bb, this.tangent1 * incrementalFriction);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb)
    {
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, this.localAnchorA);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, this.localAnchorB) : Vector3S.zero;
        return bv - av;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorA, impulse);

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            bb.angularVelocity += bb.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorB, impulse);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateSeparation(in TransformS a, in TransformS b, in f32 slop)
    {
        Vector3S worldPointA = (a.position + a.deltaPosition) + this.localAnchorA;
        Vector3S worldPointB = (b.position + b.deltaPosition) + this.localAnchorB;
        f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
        return MathS.Min(f32.zero, separation + slop);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveTwistFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Calculate relative angular velocity around the normal
        var angularVelocityA = Vector3S.Dot(a.angularVelocity, this.normal);
        var angularVelocityB = bb != null ? Vector3S.Dot(bb.angularVelocity, this.normal) : f32.zero;

        var relativeTwistVelocity = angularVelocityB - angularVelocityA;

        // Calculate twist friction impulse
        var twistImpulse = -this.twistMass * relativeTwistVelocity;

        // Limit the twist friction impulse by Coulomb's law
        var maxTwistFriction = this.accumulatedImpulse * friction;
        var newTwistFriction = MathS.Clamp(this.accumulatedTwist + twistImpulse, -maxTwistFriction, maxTwistFriction);
        twistImpulse = newTwistFriction - this.accumulatedTwist;
        this.accumulatedTwist = newTwistFriction;

        // Apply the angular impulse for twist friction
        ApplyTwistImpulse(a, bb, twistImpulse);
    }



    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyTwistImpulse(in RigidbodyS a, in RigidbodyS bb, in f32 twistImpulse)
    {
        // Apply angular impulse around the normal for twist friction
        var angularImpulse = this.normal * twistImpulse;
        a.angularVelocity -= a.tensor.inertiaWorld * angularImpulse;
        if (bb != null) bb.angularVelocity += bb.tensor.inertiaWorld * angularImpulse;
    }
}
