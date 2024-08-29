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
        if (this.accumulatedImpulse == f32.zero) return;
        if (this.accumulatedFriction == f32.zero) return;

        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Apply accumulated normal impulse
        Vector3S normalImpulse = this.normal * this.accumulatedImpulse;
        a.velocity += normalImpulse * a.inverseMass;
        a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);

        if (bb != null)
        {
            bb.velocity -= normalImpulse * bb.inverseMass;
            bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, normalImpulse);
        }

        // Apply accumulated friction impulse (correctly tangent to the contact normal)
        Vector3S frictionImpulse = this.accumulatedFriction * this.CalculateTangent(a, bb);
        a.velocity += frictionImpulse * a.inverseMass;
        a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);

        if (bb != null)
        {
            bb.velocity -= frictionImpulse * bb.inverseMass;
            bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, frictionImpulse);
        }
    }

    private Vector3S CalculateTangent(in RigidbodyS a, in RigidbodyS bb)
    {
        Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);

        if (bb != null)
        {
            contactVelocity -= bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb);
        }

        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;

        if (tangentialVelocity.sqrMagnitude < f32.epsilon)
            return Vector3S.zero;

        return tangentialVelocity.Normalize();
    }

    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool bias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        Vector3S contactVelocity = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
        if (bb != null) contactVelocity -= bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb);

        f32 vn = Vector3S.Dot(contactVelocity, this.normal);
        f32 baum = f32.zero;

        if (bias)
        {
            var separation = CalculateSeparation(a.transform.position, b.transform.position, settings.DefaultContactOffset);
            baum = settings.Baumgartner * separation * inverseDt;
        }

        f32 impulse = -this.massNormal * (vn + baum);
        f32 newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse; //Not sure, if this is saved during relaxation?

        Vector3S normalImpulse = this.normal * impulse;
        a.velocity += normalImpulse * a.inverseMass;
        a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, normalImpulse);

        if (bb != null)
        {
            bb.velocity -= normalImpulse * bb.inverseMass;
            bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, normalImpulse);
        }
    }

    public void SolveFriction(in RigidbodyS a, in Collidable b, f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        Vector3S tangent = CalculateTangent(a, bb);
        //if (tangent.sqrMagnitude < f32.epsilon) return;

        f32 invMassA = a.inverseMass;
        f32 tangentMass = invMassA + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.ra, tangent), this.ra), tangent);

        if (bb != null)
        {
            f32 invMassB = bb.inverseMass;
            tangentMass += invMassB + Vector3S.Dot(Vector3S.Cross(bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, tangent), this.rb), tangent);
        }

        f32 frictionImpulseScalar = Vector3S.Dot(tangent, tangent) / tangentMass;
        frictionImpulseScalar = -frictionImpulseScalar;

        f32 coulombMax = this.accumulatedImpulse * friction;
        f32 oldFriction = this.accumulatedFriction;
        this.accumulatedFriction = MathS.Clamp(oldFriction + frictionImpulseScalar, -coulombMax, coulombMax);
        f32 appliedFriction = this.accumulatedFriction - oldFriction;

        Vector3S frictionImpulse = tangent * appliedFriction;
        a.velocity += frictionImpulse * a.inverseMass;
        a.angularVelocity += a.tensor.inertiaWorld * Vector3S.Cross(this.ra, frictionImpulse);

        if (bb != null)
        {
            bb.velocity -= frictionImpulse * bb.inverseMass;
            bb.angularVelocity -= bb.tensor.inertiaWorld * Vector3S.Cross(this.rb, frictionImpulse);
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
