using stupid.Maths;
using System;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public Collidable a, b;
        public RigidbodyS AB, BB;
        public ContactS[] contacts;
        public readonly f32 friction, restitution;
        public readonly Vector3S averagePoint, averageNormal;

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.AB = (RigidbodyS)a;
            this.BB = b.isDynamic ? (RigidbodyS)b : null;

            this.contacts = contacts;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            var avg = Vector3S.zero;
            var nrm = Vector3S.zero;
            foreach (var c in contacts)
            {
                nrm.AddInPlace(c.normal);
                avg.AddInPlace(c.point);
            }

            this.averageNormal = (nrm / (f32)contacts.Length).Normalize();  // Ensure the normal is normalized
            this.averagePoint = avg / (f32)contacts.Length;
        }

        public void ResolveImpulse(in f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            //Solving like this, the player can jump on thing, and it wont like "roll" under due to friction
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveImpulse(deltaTime, settings, bias);
                contacts[i] = c;
            }
        }

        public void ResolveFriction(in f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveFriction(friction);
                contacts[i] = c;
            }
        }


        // PGS style resolution
        public void Resolve(in f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            //Solving like this, the player can jump on thing, and it wont like "roll" under due to friction
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveImpulse(deltaTime, settings, bias);
                contacts[i] = c;
            }


            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveFriction(friction);
                contacts[i] = c;
            }
        }
    }
}
