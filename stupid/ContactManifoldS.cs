using stupid.Maths;
using System.Runtime;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public Collidable a, b;
        public RigidbodyS ab, bb;
        public readonly f32 friction, restitution;

        public readonly ContactS c1, c2, c3, c4;
        public readonly bool isDynamicPair;

        public f32 a1, a2, a3, a4;
        public f32 f1, f2, f3, f4;
        public int contactCount;
        public void CopyToArray(ref ContactS[] array)
        {

            if (contactCount == 1)
            {
                array[0] = c1;
            }
            else if (contactCount == 2)
            {
                array[0] = c1;
                array[1] = c2;
            }
            else if (contactCount == 3)
            {
                array[0] = c1;
                array[1] = c2;
                array[2] = c3;
            }
            else if (contactCount == 4)
            {
                array[0] = c1;
                array[1] = c2;
                array[2] = c3;
                array[3] = c4;
            }
        }

        public ContactManifoldS(Collidable a, Collidable b, int contactCount, in ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.ab = a.isDynamic ? (RigidbodyS)a : null;
            this.bb = b.isDynamic ? (RigidbodyS)b : null;
            this.isDynamicPair = a.isDynamic && b.isDynamic;

            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            this.contactCount = contactCount;

            this.c1 = contacts[0];
            this.c2 = contacts[1];
            this.c3 = contacts[2];
            this.c4 = contacts[3];

            a1 = f32.zero;
            a2 = f32.zero;
            a3 = f32.zero;
            a4 = f32.zero;

            f1 = f32.zero;
            f2 = f32.zero;
            f3 = f32.zero;
            f4 = f32.zero;
        }

        void SolveDynamicPair(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            //Impulses
            if (contactCount >= 1)
            {
                c1.SolveImpulse(ab, bb, ref a1, inverseDt, settings, bias);
            }

            if (contactCount >= 2)
            {
                c2.SolveImpulse(ab, bb, ref a2, inverseDt, settings, bias);
            }

            if (contactCount >= 3)
            {
                c3.SolveImpulse(ab, bb, ref a3, inverseDt, settings, bias);
            }

            if (contactCount >= 4)
            {
                c4.SolveImpulse(ab, bb, ref a4, inverseDt, settings, bias);
            }


            //Frictions
            if (contactCount >= 1)
            {
                c1.SolveFriction(ab, bb, a1, ref f1, settings, friction);
            }

            if (contactCount >= 2)
            {
                c2.SolveFriction(ab, bb, a2, ref f2, settings, friction);
            }

            if (contactCount >= 3)
            {
                c3.SolveFriction(ab, bb, a3, ref f3, settings, friction);
            }

            if (contactCount >= 4)
            {
                c4.SolveFriction(ab, bb, a4, ref f4, settings, friction);
            }
        }

        void SolveStaticPair(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            //Impulses
            if (contactCount >= 1)
            {
                c1.SolveImpulseStatic(ab, b, ref a1, inverseDt, settings, bias);
            }

            if (contactCount >= 2)
            {
                c2.SolveImpulseStatic(ab, b, ref a2, inverseDt, settings, bias);
            }

            if (contactCount >= 3)
            {
                c3.SolveImpulseStatic(ab, b, ref a3, inverseDt, settings, bias);
            }

            if (contactCount >= 4)
            {
                c4.SolveImpulseStatic(ab, b, ref a4, inverseDt, settings, bias);
            }


            //Frictions
            if (contactCount >= 1)
            {
                c1.SolveFrictionStatic(ab, b, a1, ref f1, settings, friction);
            }

            if (contactCount >= 2)
            {
                c2.SolveFrictionStatic(ab, b, a2, ref f2, settings, friction);
            }

            if (contactCount >= 3)
            {
                c3.SolveFrictionStatic(ab, b, a3, ref f3, settings, friction);
            }

            if (contactCount >= 4)
            {
                c4.SolveFrictionStatic(ab, b, a4, ref f4, settings, friction);
            }
        }


        // PGS style resolution
        public void Resolve(in f32 inverseDt, in WorldSettings settings, in bool bias = true)
        {
            if (contactCount == 0) return;

            if (isDynamicPair)
            {
                SolveDynamicPair(inverseDt, settings, bias);
            }
            else
            {
                SolveStaticPair(inverseDt, settings, bias);
            }

        }


    }
}
