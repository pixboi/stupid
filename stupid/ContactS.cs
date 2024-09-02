using stupid.Maths;
using stupid;

public struct ContactS
{
    public readonly Vector3S point, normal, ra, rb;
    public readonly f32 massNormal;
    public readonly f32 penetrationDepth;
    public readonly int featureId;

    public f32 accumulatedImpulse, accumulatedFriction;

    public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b, int featureId = 0)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.ra = this.point - a.transform.position;
        this.rb = this.point - b.transform.position;
        this.featureId = featureId;

        this.massNormal = f32.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;

        if (a.isDynamic || b.isDynamic)
        {
            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            this.massNormal = CalculateMassNormal(ab, bb);
        }
    }

    private f32 CalculateMassNormal(RigidbodyS ab, RigidbodyS bb)
    {
        f32 invMassA = ab.inverseMass;
        f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

        f32 effectiveMass = invMassA + invMassB;

        Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
        effectiveMass += angularMassA;

        if (bb != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += angularMassB;
        }

        return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }

    public void WarmStart(in RigidbodyS a, in Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Reapply the accumulated impulses
        var normalImpulse = (this.normal * this.accumulatedImpulse);
        var tangentImpulse = CalculateTangent(a, bb) * this.accumulatedFriction;

        Vector3S warmImpulse = normalImpulse + tangentImpulse;

        ApplyImpulse(a, bb, warmImpulse);
    }

    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool bias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var baum = f32.zero;

        if (bias)
        {
            if (this.penetrationDepth > f32.zero)
            {
                baum = this.penetrationDepth * inverseDt;
            }
            else
            {
                var separation = CalculateSeparation(a.transform.position, b.transform.position, settings.DefaultContactOffset);
                baum = MathS.Max(settings.Baumgartner * separation * inverseDt, (f32)(-4f));
            }

        }

        var contactVelocity = CalculateContactVelocity(a, bb);
        var vn = Vector3S.Dot(contactVelocity, this.normal);

        var impulse = -this.massNormal * (vn + baum);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = this.normal * impulse;
        ApplyImpulse(a, bb, normalImpulse);
    }

    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Calculate tangential velocity and the corresponding impulse
        CalculateTangentialImpulse(a, bb, out var tangent, out var lambda);

        var maxFric = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + lambda, -maxFric, maxFric);
        lambda = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var finalImpulse = tangent * lambda;
        ApplyImpulse(a, bb, finalImpulse);
    }

    private Vector3S CalculateTangent(in RigidbodyS a, in RigidbodyS bb)
    {
        var contactVelocity = CalculateContactVelocity(a, bb);
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;
        var tangent = tangentialVelocity.Normalize();
        return tangent;
    }

    private void CalculateTangentialImpulse(in RigidbodyS a, in RigidbodyS bb, out Vector3S tangent, out f32 scalar)
    {
        var contactVelocity = CalculateContactVelocity(a, bb);
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;
        tangent = tangentialVelocity.Normalize();

        var invMassA = a.inverseMass;
        var tangentMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);

        if (bb != null)
        {
            var invMassB = bb.inverseMass;
            tangentMass += invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, tangent), this.rb), tangent);
        }

        scalar = -Vector3S.Dot(tangentialVelocity, tangent) / tangentMass;
    }

    public Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb)
    {
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb) : Vector3S.zero;
        return bv - av;
    }

    public void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(this.ra, impulse);

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            bb.angularVelocity += bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, impulse);
        }
    }

    private f32 CalculateSeparation(in Vector3S aPosition, in Vector3S bPosition, in f32 slop)
    {
        Vector3S worldPointA = aPosition + this.ra;
        Vector3S worldPointB = bPosition + this.rb;
        f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
        return MathS.Min(f32.zero, separation + slop);
    }
}
