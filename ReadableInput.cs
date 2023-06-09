using UnityEngine;

namespace DefaultNamespace {

    public interface ReadableInput {

        public Vector2 ReadMove();
        public float ReadAltitude();
        public float ReadSails();
        public bool ReadAim();
        
        public float ReadBrake();

    }

}