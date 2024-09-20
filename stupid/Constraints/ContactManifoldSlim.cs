using stupid.Maths;
using System.Runtime.CompilerServices;
using stupid;
using stupid.Constraints;

public struct ContactManifoldSlim
{
    public readonly Vector3S normal;
    public readonly f32 penetrationDepth;
    public readonly int contactCount;
    public ContactSlim c0, c1, c2, c3;

    public f32 accumulatedFriction, accumulatedTwist;
    public ContactManifoldSlim(RigidbodyS a, RigidbodyS b, in ContactData[] data, int contactCount)
    {
        this.normal = data[0].normal;
        this.penetrationDepth = data[0].penetrationDepth;
        this.contactCount = contactCount;

        c0 = new ContactSlim(a, b, this.normal, data[0]);
        c1 = new ContactSlim(a, b, this.normal, data[1]);
        c2 = new ContactSlim(a, b, this.normal, data[2]);
        c3 = new ContactSlim(a, b, this.normal, data[3]);

        this.accumulatedFriction = f32.zero;
        this.accumulatedTwist = f32.zero;
    }

    // Indexer with a direct array-like access pattern using unsafe pointers
    public ref ContactSlim this[int i]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (i < 0 || i >= 4) throw new System.ArgumentOutOfRangeException();
            unsafe
            {
                fixed (ContactSlim* ptr = &c0)
                {
                    return ref *(ptr + i);
                }
            }
        }
    }

    // Retain impulse data from the old manifold
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RetainData(in ContactManifoldSlim old)
    {
        if (old.contactCount == this.contactCount)
        {
            this.accumulatedFriction = old.accumulatedFriction;
            this.accumulatedTwist += old.accumulatedTwist;
        }

        for (int i = 0; i < contactCount; i++)
        {
            ref var c = ref this[i];

            for (int j = 0; j < old.contactCount; j++)
            {
                var o = old[j];
                if (Transfer(ref c, o))
                {
                    break; // Exit the inner loop once a match is found
                }
            }
        }
    }

    // Transfer impulse data from an old contact to a new one
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool Transfer(ref ContactSlim c, in ContactSlim old)
    {
        if (c.featureId == old.featureId)
        {
            c.accumulatedImpulse = old.accumulatedImpulse;
            return true;
        }
        return false;
    }

    // Warmup for iterative solvers
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Warmup(RigidbodyS a, RigidbodyS b)
    {
        for (int i = 0; i < contactCount; i++)
        {
            this[i].WarmStart(this.normal, a, b);
        }
    }

    public void Resolve(RigidbodyS a, RigidbodyS b, f32 inverseDt, WorldSettings settings, bool bias)
    {
        if (contactCount == 0) return;

        Vector3S avgPoint = Vector3S.zero;
        f32 sumAccum = f32.zero;

        if (contactCount >= 1)
        {
            c0.SolveImpulse(a, b, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            avgPoint += c0.point;
            sumAccum += c0.accumulatedImpulse;
        }

        if (contactCount >= 2)
        {
            c1.SolveImpulse(a, b, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            avgPoint += c1.point;
            sumAccum += c1.accumulatedImpulse;
        }

        if (contactCount >= 3)
        {
            c2.SolveImpulse(a, b, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            avgPoint += c2.point;
            sumAccum += c2.accumulatedImpulse;
        }

        if (contactCount >= 4)
        {
            c3.SolveImpulse(a, b, inverseDt, settings, this.penetrationDepth, this.normal, bias);
            avgPoint += c3.point;
            sumAccum += c3.accumulatedImpulse;
        }

        avgPoint /= (f32)this.contactCount;
        var friction = MathS.Sqrt(a.material.staticFriction * b.material.staticFriction);

        SolveFriction(a, b, avgPoint, sumAccum, friction);
    }

    public void SolveFriction(in RigidbodyS a, in RigidbodyS b, in Vector3S avgPoint, in f32 sumAccum, in f32 friction)
    {
        var ra = avgPoint - a.transform.position;
        var rb = b != null ? avgPoint - b.transform.position : Vector3S.zero;

        var av = a.velocity + Vector3S.Cross(a.angularVelocity, ra);
        var bv = b != null ? b.velocity + Vector3S.Cross(b.angularVelocity, rb) : Vector3S.zero;
        var contactVelocity = bv - av;

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S nv = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentVelocity = contactVelocity - nv;
        Vector3S tangent = tangentVelocity.Normalize();

        // Precompute cross products for mass calculation
        var raCross = Vector3S.Cross(ra, tangent);
        var tmass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * raCross, ra), tangent);

        if (b != null)
        {
            var rbCross = Vector3S.Cross(rb, tangent);
            tmass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * rbCross, rb), tangent);
        }

        tmass = tmass > f32.zero ? f32.one / tmass : f32.zero;

        var vt = Vector3S.Dot(contactVelocity, tangent);
        var incrementalFriction = -tmass * vt;

        var couloumbMax = sumAccum * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = tangent * incrementalFriction;
        ApplyImpulse(a, b, tangentImpulse, ra, rb);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ApplyImpulse(in RigidbodyS a, in RigidbodyS b, in Vector3S impulse, in Vector3S ra, in Vector3S rb)
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
