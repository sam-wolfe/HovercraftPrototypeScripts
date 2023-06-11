using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace DefaultNamespace {

    public interface ReadableInput {

        public Vector2 ReadMove();
        public float ReadAltitude();
        public float ReadSails();
        public bool ReadAim();
        
        public float ReadBrake();

        public event Action<InputAction.CallbackContext> OnMoveEvent;
        public event Action<InputAction.CallbackContext> OnSailEvent;
    }

}