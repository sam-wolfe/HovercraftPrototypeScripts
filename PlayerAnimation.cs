using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations.Rigging;

public class PlayerAnimation : MonoBehaviour {
    
    // TODO make interface that inputs share
    [SerializeField] private InputManager _input;
    [SerializeField] private Animator _animator;
    private static readonly int Aiming = Animator.StringToHash("Aiming");
    
    // Rig reference
    [SerializeField] private Rig _aimRig;

    void Start() {
        
    }

    void Update() {
        if (_input.aim) {
            _animator.SetBool(Aiming, true);
            // set rig weight to 1
            _aimRig.weight = 1;
        }
        else {
            _animator.SetBool(Aiming, false);
            // set rig weight to 0
            _aimRig.weight = 0;
        }
    }
}
