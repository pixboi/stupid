﻿using System;
using System.Diagnostics.Contracts;
using System.Runtime;
using stupid.Maths;

namespace stupid.Colliders
{

    public struct ContactManifoldS
    {
        public Collidable a, b;
        public RigidbodyS AB, BB;
        public ContactS[] contacts;
        public f32 friction, restitution;

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.AB = (RigidbodyS)a;
            this.BB = b.isDynamic ? (RigidbodyS)b : null;

            this.contacts = contacts;
            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;
        }

        //I think the changes need to batched here, so that one contact doesnt overpower another
        public void Resolve(in f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            SolveImpulses(deltaTime, settings, bias);
            //SolveFrictions();
            // ActuateAll();
        }

        public void SolveImpulses(in f32 deltaTime, in WorldSettings settings, bool bias = true)
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveImpulse(deltaTime, settings, bias);
                contacts[i] = c;
            }

            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.Actuate();
                contacts[i] = c;
            }

            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveFriction(friction);
                c.Actuate();

                contacts[i] = c;
            }

            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                contacts[i] = c;
            }
        }

        public void SolveFrictions()
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolveFriction(this.friction);
                //c.Actuate();
                contacts[i] = c;
            }
        }

        public void SolvePositions(in WorldSettings settings)
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.SolvePosition(settings.DefaultContactOffset, settings.PositionCorrection);
                contacts[i] = c;
            }
        }

        public void ActuateAll()
        {
            for (int i = 0; i < contacts.Length; i++)
            {
                var c = contacts[i];
                c.Actuate();
                contacts[i] = c;
            }
        }
    }
}
