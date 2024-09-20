using stupid.Maths;
using System.Runtime.CompilerServices;
using stupid;

public struct ContactSlim
{
    public readonly Vector3S point;
    public readonly byte featureId;
    public f32 normalMass, tangentMass;
    public f32 accumulatedImpulse, accumulatedFriction;
    public Vector3S tangent;

    public ContactSlim(in ContactData data)
    {
        this.point = data.point;
        this.featureId = data.featureId;
        this.accumulatedImpulse = f32.zero;
        this.normalMass = f32.zero;
        this.tangentMass = f32.zero;
        this.tangent = Vector3S.zero;
        this.accumulatedFriction = f32.zero;
    }

    public void CalculatePrestep(RigidbodyS a, RigidbodyS b, Vector3S normal)
    {
        var ra = this.point - a.transform.position;
        var rb = b != null ? this.point - b.transform.position : Vector3S.zero;

        Vector3S raCrossNormal = Vector3S.Cross(ra, normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, a.tensor.inertiaWorld * raCrossNormal);
        f32 effectiveMass = a.inverseMass + angularMassA;

        if (b != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(rb, normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, b.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += b.inverseMass + angularMassB;
        }

        this.normalMass = effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;

        // Calculate relative velocity at the contact point
        var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
        Vector3S normalVelocity = normal * Vector3S.Dot(contactVelocity, normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;

        //In retain, the previous tangent is stored IN THIS.TANGENT!
        var oldTangent = this.tangent;

        // Calculate the magnitude of the tangential velocity
        f32 tangentMag = tangentialVelocity.sqrMagnitude;
        f32 blendFactor = MathS.Clamp(tangentMag, f32.zero, f32.small);
        this.tangent = Vector3S.Lerp(oldTangent, tangentialVelocity.Normalize(), blendFactor / f32.small);

        // Precompute cross products for mass calculation
        var raCrossTangent = Vector3S.Cross(ra, this.tangent);
        var tmass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCrossTangent, ra), this.tangent);

        if (b != null)
        {
            var rbCrossTangent = Vector3S.Cross(rb, tangent);
            tmass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCrossTangent, rb), this.tangent);
        }

        this.tangentMass = tmass > f32.zero ? f32.one / tmass : f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WarmStart(in Vector3S normal, in RigidbodyS a, in RigidbodyS b)
    {
        var ra = this.point - a.transform.position;
        var rb = b != null ? this.point - b.transform.position : Vector3S.zero;

        Vector3S warmImpulse = (normal * this.accumulatedImpulse) + (this.tangent * this.accumulatedFriction);
        ApplyImpulse(a, b, warmImpulse, ra, rb);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS b, in Vector3S ra, in Vector3S rb)
    {
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
        var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
        return bv - av;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(in RigidbodyS a, in RigidbodyS b, in f32 inverseDt, in WorldSettings settings, in f32 penetrationDepth, in Vector3S normal, bool useBias = true)
    {
        f32 bias = f32.zero;
        if (penetrationDepth > f32.zero)
        {
            bias = penetrationDepth * inverseDt;
        }
        else if (useBias)
        {
            var separation = MathS.Min(f32.zero, penetrationDepth + settings.DefaultContactOffset);
            bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var ra = this.point - a.transform.position;
        var rb = b != null ? this.point - b.transform.position : Vector3S.zero;

        var contactVelocity = CalculateContactVelocity(a, b, ra, rb);
        var vn = Vector3S.Dot(contactVelocity, normal);

        var impulse = -this.normalMass * (vn + bias);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = normal * impulse;
        ApplyImpulse(a, b, normalImpulse, ra, rb);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveFriction(in RigidbodyS a, in RigidbodyS b, in f32 friction)
    {
        var ra = this.point - a.transform.position;
        var rb = b != null ? this.point - b.transform.position : Vector3S.zero;

        var contactVelocity = CalculateContactVelocity(a, b, ra, rb);

        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass * vt;

        var couloumbMax = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = this.tangent * incrementalFriction;
        ApplyImpulse(a, b, tangentImpulse, ra, rb);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(ra, impulse);

        if (b != null)
        {
            b.velocity += impulse * b.inverseMass; // B moves along normal
            b.angularVelocity += b.tensor.inertiaWorld * Vector3S.Cross(rb, impulse);
        }
    }
}
