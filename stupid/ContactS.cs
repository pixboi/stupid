using stupid.Maths;
using stupid;
using System.Runtime.CompilerServices;
using System;

public struct ContactS
{
    public readonly Vector3S point, normal, localAnchorA, localAnchorB;
    public readonly f32 penetrationDepth;
    public readonly int featureId;

    //Runtime
    public Vector3S tangent, prevTangent, ra, rb;
    public f32 normalMass, tangentMass;
    public f32 accumulatedImpulse, accumulatedFriction;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactS(in Vector3S point, in Vector3S normal, in f32 penetrationDepth, in Collidable a, in Collidable b, int featureId = -1)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.featureId = featureId;

        //This needs to be rechecked, we must compute this dir as a local dir
        //Then we transform rb and ra every iteratation into world directions!
        //We dont need to transform the initial
        this.localAnchorA = a.transform.ToLocalPoint(this.point);
        this.localAnchorB = b.transform.ToLocalPoint(this.point);
        this.ra = a.transform.TransformDirection(this.localAnchorA);
        this.rb = b.transform.TransformDirection(this.localAnchorB);

        this.normalMass = f32.zero;
        this.tangent = Vector3S.zero;
        this.prevTangent = Vector3S.zero;
        this.tangentMass = f32.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CalculatePrestep(Collidable a, Collidable b)
    {
        if (a.isDynamic || b.isDynamic)
        {
            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;
            CalculateMassNormal(ab, bb, out this.normalMass);
            CalculateTangentAndMass(ab, bb, out this.tangent, out this.tangentMass);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SubtickUpdate(Collidable a, Collidable b)
    {
        this.ra = a.transform.TransformDirection(this.localAnchorA);
        this.rb = b.transform.TransformDirection(this.localAnchorB);

        CalculatePrestep(a, b);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void CalculateMassNormal(RigidbodyS ab, RigidbodyS bb, out f32 normalMass)
    {
        Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
        f32 effectiveMass = ab.inverseMass + angularMassA;

        if (bb != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += bb.inverseMass + angularMassB;
        }

        normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void CalculateTangentAndMass(in RigidbodyS a, in RigidbodyS b, out Vector3S tangent, out f32 tangentMass)
    {
        // Calculate relative velocity at the contact point
        var contactVelocity = CalculateContactVelocity(a, b, this.ra, this.rb);

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;

        // Precompute tangential velocity magnitude and small threshold comparison
        var tangentialVelocitySqMag = tangentialVelocity.sqrMagnitude;
        bool usePrevTangent = tangentialVelocitySqMag < f32.small && this.prevTangent != Vector3S.zero;

        if (usePrevTangent)
        {
            // Blend factor between previous tangent and current tangential velocity
            f32 blendFactor = tangentialVelocitySqMag / (tangentialVelocitySqMag + f32.small);

            // Use a linear interpolation between the previous tangent and the normalized tangential velocity
            tangent = Vector3S.Lerp(this.prevTangent, tangentialVelocity.Normalize(), blendFactor);
        }
        else
        {
            tangent = tangentialVelocity.Normalize();
        }

        // Precompute cross products for mass calculation
        var raCrossTangent = Vector3S.Cross(this.ra, tangent);
        tangentMass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, this.ra), tangent);

        if (b != null)
        {
            var rbCrossTangent = Vector3S.Cross(this.rb, tangent);
            tangentMass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, this.rb), tangent);
        }

        tangentMass = tangentMass > f32.zero ? f32.one / tangentMass : f32.zero;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WarmStart(in RigidbodyS a, in Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        /*
        Vector3S warmImpulse = (this.normal * this.accumulatedImpulse) + (this.tangent * this.accumulatedFriction);
        ApplyImpulse(a, bb, warmImpulse);
        */

        var ni = this.normal;
        ni.Multiply(this.accumulatedImpulse);

        var ti = this.tangent;
        ti.Multiply(this.accumulatedFriction);

        ni.Add(ti);

        ApplyImpulse(a, bb, ni, this.ra, this.rb);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateSeparation(in TransformS a, in TransformS b, in f32 slop)
    {
        var s = this.penetrationDepth;
        s.Add(slop);
        return MathS.Min(f32.zero, s);

        /*
        var ds = b.transientPosition + this.rb - a.transientPosition - this.ra;
        //var ds = b.transientPosition;
        //ds.Add(this.rb);
        // ds.Subtract(a.transientPosition);
        //ds.Subtract(this.ra);

        f32 separation = Vector3S.Dot(ds, this.normal) + this.penetrationDepth;

        return MathS.Min(f32.zero, separation + slop);
        */
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool useBias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var bias = f32.zero;
        var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);

        if (separation > f32.zero)
        {
            bias = separation * inverseDt;
        }
        else if (useBias)
        {
            bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var contactVelocity = CalculateContactVelocity(a, bb, this.ra, this.rb);
        var vn = Vector3S.Dot(contactVelocity, this.normal);

        var impulse = -this.normalMass * (vn + bias);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = this.normal * impulse;
        ApplyImpulse(a, bb, normalImpulse, this.ra, this.rb);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 inverseDt, in f32 friction, in WorldSettings settings, in f32 sumAccum, bool useBias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var contactVelocity = CalculateContactVelocity(a, bb, this.ra, this.rb);

        /*
        //var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);
        //f32 bias = f32.zero;
        // if (useBias) bias = settings.Baumgartner * separation * inverseDt;
        */

        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass;
        incrementalFriction.Multiply(vt);

        var couloumbMax = sumAccum * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = this.tangent * incrementalFriction;
        ApplyImpulse(a, bb, tangentImpulse, this.ra, this.rb);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb, in Vector3S ra, in Vector3S rb)
    {
        /*
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb) : Vector3S.zero;
        return bv - av;
        */

        var ai = a.velocity;
        var ac = a.angularVelocity;
        ac.CrossInPlace(ra);// Vector3S.Cross(a.angularVelocity, this.ra);
        ai.Add(ac);

        var bi = Vector3S.zero;
        if (bb != null)
        {
            bi = bb.velocity;
            var bc = bb.angularVelocity; //Vector3S.Cross(bb.angularVelocity, this.rb);
            bc.CrossInPlace(rb);
            bi.Add(bc);
        }

        bi.Subtract(ai);
        return bi;

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
    {
        /*
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(this.ra, impulse);

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            bb.angularVelocity += bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, impulse);
        }

        */
        // Update linear velocity for 'a'
        Vector3S ai = impulse;
        ai.Multiply(a.inverseMass); // ai = impulse * a.inverseMass
        a.velocity.Subtract(ai); // a.velocity += ai

        // Update angular velocity for 'a'
        Vector3S raCrossImpulse = ra;
        raCrossImpulse.CrossInPlace(impulse);// Vector3S.Cross(this.ra, impulse); // raCrossImpulse = Cross(ra, impulse)
        raCrossImpulse.Multiply(a.tensor.inertiaWorld); // raCrossImpulse = a.tensor.inertiaWorld * raCrossImpulse
        a.angularVelocity.Subtract(raCrossImpulse); // a.angularVelocity -= raCrossImpulse

        // If 'bb' is not null, update linear and angular velocities for 'bb'
        if (bb != null)
        {
            // Update linear velocity for 'bb'
            Vector3S bi = impulse;
            bi.Multiply(bb.inverseMass); // bi = impulse * bb.inverseMass
            bb.velocity.Add(bi); // bb.velocity += bi

            // Update angular velocity for 'bb'
            Vector3S rbCrossImpulse = rb;
            rbCrossImpulse.CrossInPlace(impulse);//Vector3S.Cross(this.rb, impulse); // rbCrossImpulse = Cross(rb, impulse)
            rbCrossImpulse.Multiply(bb.tensor.inertiaWorld); // rbCrossImpulse = bb.tensor.inertiaWorld * rbCrossImpulse
            bb.angularVelocity.Add(rbCrossImpulse); // bb.angularVelocity += rbCrossImpulse
        }


    }



}
