using System;
using System.Collections;
using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour, ReadableInput {

    public float altitude { get; private set; }
    public Vector2 move { get; private set; }
    public float sails { get; private set; }
    public bool aim { get; private set; }
    public float lateralBrake { get; private set; }
    
    private KMInput _input;
    private InputAction movement;
    private InputAction sail;
    
    public void OnAltitude(InputAction.CallbackContext context) {
        float newAltitude = context.ReadValue<float>();

        altitude = newAltitude;
        // Debug.Log(altitude);
    }
    
    public event Action<InputAction.CallbackContext> OnMoveEvent;
    public event Action<InputAction.CallbackContext> OnSailEvent;
    
    // public void HandleOnMove(InputAction.CallbackContext context) {
    // public void HandleOnMove(Action<InputAction.CallbackContext> onMoveAction) {
        // Vector2 newMovement = context.ReadValue<Vector2>();

        // move = newMovement;
        // Debug.Log(move);
        
        // movement = _input.Car.Move;
        // movement.performed += context => {
        //     move = context.ReadValue<Vector2>();
        //     Debug.Log("Movement: " + move);
        // };
        // movement.Enable();
        
        //--------------------------------
        
        // movement = _input.Car.Move;
        // movement.performed += onMoveAction;
        // movement.canceled += onMoveAction;
        // movement.Enable();
    // }
    
    void Awake() {
        _input = new KMInput();
        
        movement = _input.Car.Move;
        movement.performed += context => OnMoveEvent?.Invoke(context);
        movement.canceled += context => OnMoveEvent?.Invoke(context);
        movement.Enable();
        
        sail = _input.Car.Sail;
        sail.performed += context => OnSailEvent?.Invoke(context);
        sail.canceled += context => OnSailEvent?.Invoke(context);
        sail.Enable();
    }

    // private void OnEnable() {
    //     movement = _input.Car.Move;
    //     movement.performed += context => {
    //         move = context.ReadValue<Vector2>();
    //         Debug.Log("Movement: " + move);
    //     };
    //     movement.canceled += context => {
    //         move = context.ReadValue<Vector2>();
    //         Debug.Log("Movement: " + move);
    //     };
    //     movement.Enable();
    // }

    public void OnSail(InputAction.CallbackContext context) {
        sails = context.ReadValue<float>();;
        // Debug.Log(sails);
    }
    
    public void OnLateralBrake(InputAction.CallbackContext context) {
        lateralBrake = context.ReadValue<float>();
    }
    
    public void OnAim(InputAction.CallbackContext context) {
        switch (context.phase) {
            case InputActionPhase.Started:
                aim = true;
                break;
            case InputActionPhase.Performed:
                return;
            case InputActionPhase.Canceled:
                aim = false;
                break;
        }
    }

    public Vector2 ReadMove() {
        return move;
    }

    public float ReadAltitude() {
        return altitude;
    }

    public float ReadSails() {
        return sails;
    }

    public bool ReadAim() {
        return aim;
    }
    
    public float ReadBrake() {
        return lateralBrake;
    }

}
