using stupid.Maths;
using System.Collections.Generic;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public List<ContactS> contacts;

        public ContactManifoldS(List<ContactS> contacts)
        {
            this.contacts = contacts;
        }
    }
}
