using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CustomActionInputManager : MonoBehaviour {
    
    private KMInput _input;
    private InputAction movement;

    void Awake() {
        _input = new KMInput();
    }

    // private void OnEnable() {
    //     movement = _input.Car.Move;
    //     movement.performed += context => Debug.Log("Movement: " + context.ReadValue<Vector2>());
    //     movement.Enable();
    // }

    // void FixedUpdate() {
    //     Debug.Log("Movement: " + movement.ReadValue<Vector2>());
    // }
}
