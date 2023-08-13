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
    
    [Header("Aim Settings")]
    private Vector3 rotationVelocity;

    [SerializeField] private GameObject aimIKTarget;
    [SerializeField] private Camera mainCamera;
    
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
    
    // PID settings
    [SerializeField] private float pTerm = 0f;
    [SerializeField] private float iTerm = 0f;
    [SerializeField] private float dTerm = 0f;
    
    
    [SerializeField] private float _uprightStrength = 10f;
    [SerializeField] private float _uprightStrengthDamper = 0.5f;
    
    // CM Virtual camera to switch to when aiming
    [SerializeField] private CinemachineFreeLook _aimingCamera;
    
    public Vector3 targetAltitude { get; private set; }
    
    [SerializeField] private AudioClip _devAudio1;
    [SerializeField] private AudioSource _audioSource;

    [SerializeField] private float _maxParticleDistance = 5f;

    [Header("References")] 
    // TODO extract to a new class for handling particles 
    [SerializeField]
    private ParticleSystem _idleParticles;
    [SerializeField] private ParticleSystem _motionParticlesA;
    [SerializeField] private ParticleSystem _motionParticlesB;
    
    [Header("Movement Settings")]
        
    [SerializeField] public Gyro _gyro;
    [SerializeField] public Booster _booster;
    [SerializeField] public Fan _fan;
    [SerializeField] public PressureVent _pressureVent;
    [SerializeField] public Hydrolic _hydrolic;

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


    private PIDController _pid = new();
    
    private bool resetBoostFlag = false;
    private bool resetViewFlag = false;
    bool resetLatBrakeFlag = false;

    
    private void Start() {
        rb = GetComponent<Rigidbody>();
        targetAltitude = transform.position;
        
        Cursor.lockState = CursorLockMode.Locked;
        
        _pressureVent.brakeForce = _pressureVent.maxBrakeForce;
        
        if (_playerInput != null) {
            _input = _playerInput;
        } else if (_enemyAi != null) {
            _input = _enemyAi;
        } else {
            Debug.LogError("No input assigned to hovercar controller");
        }
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
        
        // Place _aimIKTarget 20 units in front of the camera
        aimIKTarget.transform.position = mainCamera.transform.position + mainCamera.transform.forward * 20;
        
        updateTargetAltitue();
        moveShipHorizonal();
        moveToTargetAltitude();
        rotateSails();
        UpdateUprightForce();
        UpdateLateralBreak();
        UpdateBoost();
        UpdateBrake();

        FindIdleParticleSpawn();
        FindMotionParticleSpawn();
    }

    private void FindMotionParticleSpawn() {
        // If velocity is > 1, raycast down to find the ground level and move _motionParticles to that y position and start.
        // _motionParticles should be a child of the hovercar so it moves with it, so the x and z don't need to be modified.
        if (rb.velocity.magnitude > 1) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, Vector3.down, out hit, _maxParticleDistance)) {
                _motionParticlesA.transform.position = new Vector3(_motionParticlesA.transform.position.x, hit.point.y, _motionParticlesA.transform.position.z);
                _motionParticlesB.transform.position = new Vector3(_motionParticlesB.transform.position.x, hit.point.y, _motionParticlesB.transform.position.z);
                _motionParticlesA.transform.localPosition = new Vector3(0, _motionParticlesA.transform.localPosition.y, 10f);
                _motionParticlesB.transform.localPosition = new Vector3(0, _motionParticlesB.transform.localPosition.y, 10f);
                // Only play if not already playing
                if (!_motionParticlesA.isPlaying) {
                    _motionParticlesA.Play();
                    _motionParticlesB.Play();
                }
            } else {
                // If no ground is found, stop playing the particles
                _motionParticlesA.Stop();
                _motionParticlesB.Stop();
            }
        }
        else {
            _motionParticlesA.Stop();
            _motionParticlesB.Stop();
        }
    }

    private void FindIdleParticleSpawn() {
        // If velocity is < 1, raycast down to find the ground and move _idleParticles to that position and start
        // playing the particles
        if (rb.velocity.magnitude < 1) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, Vector3.down, out hit, 100)) {
                _idleParticles.transform.position = hit.point;
                // Only play if not already playing
                if (!_idleParticles.isPlaying) {
                    _idleParticles.Play();
                }
            }
        }
        else {
            _idleParticles.Stop();
        }
    }
    

    private void UpdateBoost() {
        // Read boost input and apply to forward momentum
        if (boost) {
            rb.AddForce(transform.forward * _booster.boostForce, ForceMode.Force);
            if (resetBoostFlag) {
                _boostImpulse.GenerateImpulse();
                resetBoostFlag = false;
                _audioSource.PlayOneShot(_devAudio1);
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
            
            rb.AddForce(-transform.right * lateralBrake *_pressureVent.brakeForce, ForceMode.Force);
            if (resetLatBrakeFlag) {
                _LeftBrakeImpulse.GenerateImpulse();
                resetLatBrakeFlag = false;
            }
        } else if (lateralBrake < 0) {
            rb.AddForce(transform.right * -lateralBrake * _pressureVent.brakeForce, ForceMode.Force);
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
            _pressureVent.brakeForce = Mathf.MoveTowards(
                _pressureVent.brakeForce, 
                0, 
                _pressureVent.breakDepleteRate * Time.deltaTime
            );
        } else {
            _pressureVent.brakeForce = Mathf.MoveTowards(
                _pressureVent.brakeForce, 
                _pressureVent.maxBrakeForce, 
                _pressureVent.breakDepleteRate / _pressureVent.breakRefilRate * Time.deltaTime
            );
        }
    }
    
    
    private void depleteBoostGuage() {
        if (boost) {
            _booster.boostForce = Mathf.MoveTowards(
                _booster.boostForce, 
                0, 
                _booster.boostDepleteRate * Time.deltaTime
            );
        } else {
            _booster.boostForce = Mathf.MoveTowards(
                _booster.boostForce, 
                _booster.maxBoostForce, 
                _booster.boostDepleteRate / _booster.boostRefillRate * Time.deltaTime
            );
        }
    }

    
    private void moveShipHorizonal() {
        Vector3 forwardForce = transform.forward * move.y;
        
        Vector3 sideForce = transform.right * move.x;

        Vector3 direction = forwardForce + sideForce;
        
        rb.AddForce(direction * _fan.accelerationRate, ForceMode.Force);
        
        // Limit speed
        if (rb.velocity.magnitude > _fan.maxSpeed) {
            rb.velocity = rb.velocity.normalized * _fan.maxSpeed;
        }
        
        if (move.x == 0 && move.y == 0) {
            
            // When there is no input on the move vector, slowly kill horizontal velocity, but allow vertical velocity
            // to continue increasing using the Mathf.MoveTowards function.
            Vector3 newVelocity = new Vector3(
                Mathf.MoveTowards(rb.velocity.x, 0, _hydrolic.carDrag * Time.deltaTime), 
                rb.velocity.y, 
                Mathf.MoveTowards(rb.velocity.z, 0, _hydrolic.carDrag * Time.deltaTime)
            );
            
            rb.velocity = newVelocity;
            
        }
    }

    
    private void rotateSails() {
        // It's called sails because this used to be a boat controller
        if (sails != 0 && aim == false) {
            rb.AddTorque(Vector3.up * (sails * _gyro.turnRate * Time.deltaTime), ForceMode.Force);

        } else {
            
            // When no input is read on sails i.e. "0" the ship will slowly stop turning
            rb.AddTorque(-rb.angularVelocity * _gyro.turnDrag, ForceMode.Force);
        }
    }
    
    
    private void processAim() {
        // Aiming is mostly handled directly by cinemachine.
        // If not aiming, rotate to default rotation

        Vector3 targetEuler;
        if (aim) {

            if (resetViewFlag) {
                // Reset view to default rotation when aim is first pressed after not aiming.
                // This is to prevent the aim camera from jumping when aim is released. 
                // targetRotation = transform.localEulerAngles;
                _aimingCamera.m_XAxis.Value = 0;
                _aimingCamera.m_YAxis.Value = 0.35f;
                resetViewFlag = false;
            }
            
        } else {
            resetViewFlag = true;
        }
        
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
            Vector3 newThrust = new Vector3(
                0, Mathf.Clamp(input, localMin, _fan.verticalAcceleration), 0);
            //------------------------------------------------------------

            rb.AddForce(newThrust, ForceMode.Force);
        }
    }


    private void updateTargetAltitue() {
        var shipPosition = transform.position;
        
        targetAltitude = new Vector3(
                shipPosition.x, 
                targetAltitude.y + (altitude * _fan.targetAltitudeSpeed * Time.deltaTime), 
                shipPosition.z
            );
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.down), out hit, Mathf.Infinity)) {
            targetAltitude = new Vector3(0, (transform.position.y - hit.distance) + _gyro.hovercarRestHeight, 0);
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
