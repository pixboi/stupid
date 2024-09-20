using stupid.Maths;
using System.Runtime.CompilerServices;
using stupid;


//The only thing different really, between contacts that share the same manifold, are the offset FROM A, and feature ID
//We could grealt reduce the memory footprint with this... i think

public readonly struct ContactData
{
    //Init
    public readonly Vector3S point, normal;
    public readonly f32 penetrationDepth;
    public readonly byte featureId;
}

public struct ContactS
{
    //Init
    public readonly Vector3S point, normal, ra, rb;
    public readonly f32 penetrationDepth;
    public readonly byte featureId;

    //Runtime
    public Vector3S tangent, prevTangent;
    public f32 normalMass, tangentMass;
    public f32 accumulatedImpulse, accumulatedFriction;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactS(in Vector3S point, in Vector3S normal, in f32 penetrationDepth, in Collidable a, in Collidable b, byte featureId = 255)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.featureId = featureId;

        var localAnchorA = a.transform.ToLocalPoint(this.point);
        var localAnchorB = b.transform.ToLocalPoint(this.point);
        this.ra = a.transform.TransformDirection(localAnchorA);
        this.rb = b.transform.TransformDirection(localAnchorB);

        this.normalMass = f32.zero;
        this.tangent = Vector3S.zero;
        this.tangentMass = f32.zero;
        this.prevTangent = Vector3S.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CalculatePrestep(RigidbodyS ab, Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;
        CalculateMassNormal(ab, bb, out this.normalMass);
        CalculateMassTangent(ab, bb, out this.tangent, out this.tangentMass);
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
    void CalculateMassTangent(in RigidbodyS a, in RigidbodyS b, out Vector3S tangent, out f32 tangentMass)
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

        Vector3S warmImpulse = (this.normal * this.accumulatedImpulse) + (this.tangent * this.accumulatedFriction);
        ApplyImpulse(a, bb, warmImpulse, this.ra, this.rb);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool useBias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        f32 bias = f32.zero;

        if (this.penetrationDepth > f32.zero)
        {
            bias = this.penetrationDepth * inverseDt;
        }
        else if (useBias)
        {
            var separation = CalculateSeparation(a.transform, b.transform, this.penetrationDepth, settings.DefaultContactOffset);
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
    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 inverseDt, in f32 friction, in WorldSettings settings, bool useBias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var contactVelocity = CalculateContactVelocity(a, bb, this.ra, this.rb);

        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass * vt;

        var couloumbMax = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = this.tangent * incrementalFriction;
        ApplyImpulse(a, bb, tangentImpulse, this.ra, this.rb);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolvePatchFriction(in RigidbodyS a, in Collidable b, in Vector3S avgPoint, in f32 sumAccum, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var RA = avgPoint - a.transform.position;
        var RB = avgPoint - b.transform.position;
        var contactVelocity = CalculateContactVelocity(a, bb, RA, RB);

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S nv = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tv = contactVelocity - nv;
        Vector3S t = tv.Normalize();

        // Precompute cross products for mass calculation
        var ract = Vector3S.Cross(RA, t);
        var tmass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * ract, RA), t);

        if (bb != null)
        {
            var rbct = Vector3S.Cross(RB, t);
            tmass += bb.inverseMass + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * rbct, RB), t);
        }

        tmass = tmass > f32.zero ? f32.one / tmass : f32.zero;

        var vt = Vector3S.Dot(contactVelocity, t);
        var incrementalFriction = -tmass * vt;

        var couloumbMax = sumAccum * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = t * incrementalFriction;
        ApplyImpulse(a, bb, tangentImpulse, RA, RB);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb, in Vector3S ra, in Vector3S rb)
    {
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, rb) : Vector3S.zero;
        return bv - av;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            bb.angularVelocity += bb.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static f32 CalculateSeparation(in TransformS a, in TransformS b, in f32 pen, in f32 slop)
    {
        return MathS.Min(f32.zero, pen + slop);

        /*
        var ds = b.transientPosition + this.rb - a.transientPosition - this.ra;
        f32 separation = Vector3S.Dot(ds, this.normal) + this.penetrationDepth;
        return MathS.Min(f32.zero, separation + slop);
        */
    }



}
