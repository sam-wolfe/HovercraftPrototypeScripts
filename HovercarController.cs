using System;
using Cinemachine;
using DefaultNamespace;
using UnityEngine;

public class HovercarController : MonoBehaviour {

    
    // ------------------------------------------------
    // Notes:
    //  Drag is currently set to 1 for drag and angular
    //  drag, so when moving to a new scene dont forget
    //  to update those in the inspector if they aren't
    //  a parameter here.
    //
    // ------------------------------------------------
    
    [Header("Movement Settings")]
        
    [SerializeField] private Gyro _gyro;
    [SerializeField] private Booster _booster;
    [SerializeField] private Fan _fan;
    [SerializeField] private PressureVent _pressureVent;
    [SerializeField] private Hydrolic _hydrolic;
    
    [SerializeField]
    [Range(1, 100)]
    // On Fan
    private float accelerationRate = 10f;
    
    [SerializeField]
    [Tooltip("Maximum horizontal speed of hovercar")]
    [Range(1, 100)]
    // On Fan
    private float maxSpeed = 40f;
    
    [SerializeField] [Tooltip("Rate the hovercar will stop moving when no input is given")] 
    [Range(0.1f, 100f)]
    // On Hydrolic
    private float carDrag = 20f;
    
    [SerializeField]
    [Tooltip("How fast the ship can change altitude, affected by hovercar mass.")]
    [Range(1, 200)]
    // On Fan
    private float verticalAcceleration = 40f;
    
    [SerializeField]
    [Tooltip("Idle altitude of hovercar.")]
    [Range(1, 10)]
    // On Gyro
    private float hovercarRestHeight = 2f;
    
    // private float minSpeed = 40f;
    [SerializeField]
    [Tooltip("How fast the ship can change direction, normally.")]
    [Range(1, 800)]
    // On Gryo
    private float turnRate = 100f;

    [SerializeField] [Tooltip("Rate the hovercar will stop turning when no input is given")] 
    [Range(0.1f, 40f)]
    // On Gryo
    private float turnDrag = 10f;
    
    [SerializeField]
    // On PressureVent
    private float brakeForce = 500f;
    [SerializeField]
    [Range(100, 1000)]
    // On PressureVent
    private float maxBrakeForce = 500f;

    
    [SerializeField]
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 10)]
    // On PressureVent
    private float breakRefilRate = 4f;

    [SerializeField] 
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 2000)]
    // On PressureVent
    private float breakDepleteRate = 800f;
    
    [SerializeField]
    [Tooltip("Boost force applied to hovercar when boost is active.")]
    [Range(1, 1000)]
    // On Booster
    private float boostForce = 500f;
    
    [SerializeField]
    [Range(100, 1000)]
    // On Booster
    private float maxBoostForce = 500f;
    
    [SerializeField]
    [Tooltip("Rate the hovercars Boost force will regenerate, lower=faster.")] 
    [Range(1, 100)]
    // On Booster
    private float boostRefilRate = 4f;
    
    [SerializeField] 
    [Tooltip("Rate the hovercars Boost force will regenerate, lower=faster.")] 
    [Range(1, 2000)]
    // On Booster
    private float boostDepleteRate = 800f;    

    [Header("Aim Settings")]
    [SerializeField]
    [Tooltip("Gameobject that the camera uses to rotate around when aiming guns.")]
    private GameObject aimTarget;
    [SerializeField] private float xAimRate = 100f;
    [SerializeField] private float yAimRate = 100f;
    [SerializeField] private float aimTurnDamper = 0.5f;
    private Vector3 AIM_TARGET_DEFAULT_ROTATION = new Vector3(0, 0, 0);
    
    // public float aimSmoothTime = 0.1f;
    public float aimSmoothTime = 5f;
    private Vector3 aimTargetDefaultRotation;
    private Vector3 targetRotation;
    private Vector3 rotationVelocity;
    [SerializeField] private GameObject aimIKTarget;
    
    [Header("Impulses")]
    [SerializeField]
    private CinemachineImpulseSource _boostImpulse;
    [SerializeField]
    private CinemachineImpulseSource _LeftBrakeImpulse;
    [SerializeField]
    private CinemachineImpulseSource _RightBrakeImpulse;
    
    [Header("Input")]

    // TODO make interface that inputs share
    [SerializeField] private InputManager _playerInput;

    [SerializeField] private EnemyAI _enemyAi;
    
    private ReadableInput _input;

    [Header("Dev")]
    // On fan targetAltitudeSpeed
    [SerializeField] private float devTargetAltitudeSpeed = 5f;
    
    // PID settings
    [SerializeField] private float pTerm = 0f;
    [SerializeField] private float iTerm = 0f;
    [SerializeField] private float dTerm = 0f;
    
    
    [SerializeField] private float _uprightStrength = 10f;
    [SerializeField] private float _uprightStrengthDamper = 0.5f;
    
    // CM Virtual camera to switch to when aiming
    [SerializeField] private CinemachineVirtualCamera _aimingCamera;

    private Vector2 move;
    private float altitude;
    private float sails;
    private float lateralBrake;
    private bool aim;
    private bool fire;
    private bool boost;
    private bool brake;
    private Vector2 gunAim;

    private Rigidbody rb;

    public Vector3 targetAltitude { get; private set; }

    private PIDController _pid = new();
    
    private bool resetBoostFlag = false;
    private bool resetViewFlag = false;
    bool resetLatBrakeFlag = false;

    
    private void Start() {
        rb = GetComponent<Rigidbody>();
        targetAltitude = transform.position;
        
        Cursor.lockState = CursorLockMode.Locked;
        
        brakeForce = maxBrakeForce;
        
        if (_playerInput != null) {
            _input = _playerInput;
        } else if (_enemyAi != null) {
            _input = _enemyAi;
        } else {
            Debug.LogError("No input assigned to hovercar controller");
        }
        
        // Store the default rotation at the start of the script
        aimTargetDefaultRotation = aimTarget.transform.localEulerAngles;
    }

    
    void Update() {

        updatePIDSettings();
        
        move = _input.ReadMove();
        altitude = _input.ReadAltitude();
        sails = _input.ReadSails();
        lateralBrake = _input.ReadLatBrake();
        aim = _input.ReadAim();
        fire = _input.ReadFire();
        boost = _input.ReadBoost();
        brake = _input.ReadBrake();
        gunAim = _input.ReadGunAim();

        // wtf this is happening every frame, it should only happen when the player presses the button
        if (_aimingCamera != null) {
            if (aim) {
                _aimingCamera.Priority = 11;
            }
            else {
                _aimingCamera.Priority = 9;
            }
        }

        processAim();

    }

    
    private void updatePIDSettings() {
        _pid.proportionalGain = pTerm;
        _pid.integralGain = iTerm;
        _pid.derivitiveGain = dTerm;
        
    }

    
    private void FixedUpdate() {
        updateTargetAltitue();
        moveShipHorizonal();
        moveToTargetAltitude();
        rotateSails();
        UpdateUprightForce();
        UpdateLateralBreak();
        UpdateBoost();
        UpdateBrake();
    }
    

    private void UpdateBoost() {
        // Read boost input and apply to forward momentum
        if (boost) {
            rb.AddForce(transform.forward * boostForce, ForceMode.Force);
            if (resetBoostFlag) {
                _boostImpulse.GenerateImpulse();
                resetBoostFlag = false;
            }
        }
        else {
            resetBoostFlag = true;
        }
        
        // Deplete and regenerate boost
        depleteBoostGuage();
    }
    
    
    private void UpdateBrake() {
        // Read brake and nullify all momentum over time
        
        if (brake) {
            rb.AddForce(-rb.velocity * Time.deltaTime * 100, ForceMode.Force);
        }
        
    }
    
    
    private void UpdateLateralBreak() {
        depleteBreakGuage();
        
        if (lateralBrake > 0) {
            //Apply force until sideways velocity is 0 but retain forward and backward velocity
            
            rb.AddForce(-transform.right * lateralBrake * brakeForce, ForceMode.Force);
            if (resetLatBrakeFlag) {
                _LeftBrakeImpulse.GenerateImpulse();
                resetLatBrakeFlag = false;
            }
        } else if (lateralBrake < 0) {
            rb.AddForce(transform.right * -lateralBrake * brakeForce, ForceMode.Force);
            if (resetLatBrakeFlag) {
                _RightBrakeImpulse.GenerateImpulse();
                resetLatBrakeFlag = false;
            }
        } else {
            resetLatBrakeFlag = true;
        }
    }

    
    private void depleteBreakGuage() {
        // while lateralBreak != 0, reduce breakForce to 0 over 1 second.
        // when lateralBreak == 0, increase breakForce to max over 10 seconds.
        // TODO make this a setting
        if (lateralBrake != 0) {
            brakeForce = Mathf.MoveTowards(brakeForce, 0, breakDepleteRate * Time.deltaTime);
        } else {
            brakeForce = Mathf.MoveTowards(brakeForce, maxBrakeForce, breakDepleteRate / breakRefilRate * Time.deltaTime);
        }
    }
    
    
    private void depleteBoostGuage() {
        if (boost) {
            boostForce = Mathf.MoveTowards(boostForce, 0, boostDepleteRate * Time.deltaTime);
        } else {
            boostForce = Mathf.MoveTowards(boostForce, maxBoostForce, boostDepleteRate / boostRefilRate * Time.deltaTime);
        }
    }

    
    private void moveShipHorizonal() {
        Vector3 forwardForce = transform.forward * move.y;
        
        Vector3 sideForce = transform.right * move.x;

        Vector3 direction = forwardForce + sideForce;
        
        rb.AddForce(direction * accelerationRate, ForceMode.Force);
        
        // Limit speed
        if (rb.velocity.magnitude > maxSpeed) {
            rb.velocity = rb.velocity.normalized * maxSpeed;
        }
        
        if (move.x == 0 && move.y == 0) {
            
            // When there is no input on the move vector, slowly kill horizontal velocity, but allow vertical velocity
            // to continue increasing using the Mathf.MoveTowards function.
            Vector3 newVelocity = new Vector3(
                Mathf.MoveTowards(rb.velocity.x, 0, carDrag * Time.deltaTime), 
                rb.velocity.y, 
                Mathf.MoveTowards(rb.velocity.z, 0, carDrag * Time.deltaTime)
            );
            
            rb.velocity = newVelocity;
            
        }
    }

    
    private void rotateSails() {
        // It's called sails because this used to be a boat controller
        if (sails != 0 && aim == false) {
            rb.AddTorque(Vector3.up * (sails * turnRate * Time.deltaTime), ForceMode.Force);

        } else {
            
            // When no input is read on sails i.e. "0" the ship will slowly stop turning
            rb.AddTorque(-rb.angularVelocity * turnDrag, ForceMode.Force);
        }
    }
    
    
    private void processAim() {
        // If aiming, when `sails` input is not 0, rotate `aimTarget` by that amount
        // If not aiming, rotate to default rotation

        Vector3 targetEuler;
        if (aim) {

            if (resetViewFlag) {
                // Reset view to default rotation when aim is first pressed after not aiming.
                // This is to prevent the aim camera from jumping when aim is released. 
                targetRotation = transform.localEulerAngles;
                targetEuler = new Vector3(0,0,0);
                aimTarget.transform.localEulerAngles = targetEuler;
                resetViewFlag = false;
            }
            
            // Calculate the desired rotation
            aimTarget.transform.rotation *= Quaternion.AngleAxis(gunAim.x * xAimRate, Vector3.up);
            aimTarget.transform.rotation *= Quaternion.AngleAxis(gunAim.y * yAimRate, Vector3.right);
            
            var angles = aimTarget.transform.localEulerAngles;
            angles.z = 0;
            
            var angle = aimTarget.transform.localEulerAngles.x;

            bool blockQ = false;
            
           // Clamp rotation to ~180 degrees in front of pilot
            if (angles.y >= 180 && angles.y < 240) {
                angles.y = 240;
            } else if (angles.y > 90 && angles.y < 180) {
                angles.y = 90;
            }
            
            //Clamp the Up/Down rotation
            if (angle > 180 && angle < 340) {
                angles.x = 340;
            }
            else if(angle < 180 && angle > 40) {
                angles.x = 40;
            }

            aimTarget.transform.localEulerAngles = angles;

        } else {
            resetViewFlag = true;
        }
        
        // set aimIKTarget transform to 5.73 units in front of the aimTarget, do not change y axis of aimIKTarget
        Vector3 newPosition = 
            aimTarget.transform.position + aimTarget.transform.forward * 5.73f - aimTarget.transform.right * -0.5f;
        
        aimIKTarget.transform.position = 
            new Vector3(newPosition.x, newPosition.y, newPosition.z);

    }

    private void moveToTargetAltitude() {

        // We only want the difference in altitute between target and 
        // current position, so zero out the other axis's (axees? axies?)
        float currentHeight = transform.position.y;
        var targetHeight = targetAltitude.y;


        float input = _pid.Update(Time.deltaTime, currentHeight, targetHeight);
        
        if (input > 0) {
            
            // Not related to base PID controller function, limits ascent speed
            // Warning: Unintentially related to PID controller. If the verticalAcceleration is too low
            // the PID controller will not be able to reach the target altitude.
            var localMin = float.MinValue;
            Vector3 newThrust = new Vector3(0, Mathf.Clamp(input, localMin, verticalAcceleration), 0);
            //------------------------------------------------------------

            rb.AddForce(newThrust, ForceMode.Force);
        }
    }


    private void updateTargetAltitue() {
        var shipPosition = transform.position;
        
        targetAltitude = new Vector3(
                shipPosition.x, 
                targetAltitude.y + (altitude * devTargetAltitudeSpeed * Time.deltaTime), 
                shipPosition.z
            );
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.down), out hit, Mathf.Infinity)) {
            targetAltitude = new Vector3(0, (transform.position.y - hit.distance) + hovercarRestHeight, 0);
        }
    }
    

    private Quaternion ShortestRotation(Quaternion to, Quaternion from) {
        if (Quaternion.Dot(to, from) < 0) {
            return to * Quaternion.Inverse(Multiply(from, -1));
        }
        else {
            return to * Quaternion.Inverse(from);
        }
    }
    
    
    private Quaternion Multiply(Quaternion input, float scalar)
    {
        return new Quaternion(input.x * scalar, input.y * scalar, input.z * scalar, input.w * scalar);
    }

    
    public void UpdateUprightForce() {
        Quaternion currentRotation = transform.rotation;
        Vector3 targetRotVec = new Vector3(transform.forward.x, 0f, transform.forward.z);
        Quaternion targetRotation = Quaternion.LookRotation(targetRotVec, Vector3.up);
        Quaternion toGoal = ShortestRotation(targetRotation, currentRotation);
        
        float rotDegrees;
        Vector3 rotAxis;
        toGoal.ToAngleAxis(out rotDegrees, out rotAxis);
        
        rotAxis.Normalize();

        float rotRadians = rotDegrees * Mathf.Deg2Rad;
        
        rb.AddTorque((rotAxis * (rotRadians * _uprightStrength))- (rb.angularVelocity * _uprightStrengthDamper));
    }

}
