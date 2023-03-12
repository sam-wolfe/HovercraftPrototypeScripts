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
    [SerializeField]
    [Range(1, 100)]
    private float accelerationRate = 10f;
    
    [SerializeField]
    [Tooltip("Maximum horizontal speed of hovercar")]
    [Range(1, 100)]
    private float maxSpeed = 40f;
    
    [SerializeField] [Tooltip("Rate the hovercar will stop moving when no input is given")] 
    [Range(0.1f, 100f)]
    private float carDrag = 20f;
    
    [SerializeField]
    [Tooltip("How fast the ship can change altitude, affected by hovercar mass.")]
    [Range(1, 200)]
    private float verticalAcceleration = 40f;
    
    // private float minSpeed = 40f;
    [SerializeField]
    [Tooltip("How fast the ship can change direction, normally.")]
    [Range(1, 800)]
    private float turnRate = 100f;

    [SerializeField] [Tooltip("Rate the hovercar will stop turning when no input is given")] 
    [Range(0.1f, 40f)]
    private float turnDrag = 10f;
    
    [SerializeField]
    private float brakeForce = 500f;
    [SerializeField]
    [Range(100, 1000)]
    private float maxBrakeForce = 500f;
    
    [SerializeField]
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 10)]
    private float breakRefilRate = 4f;

    [SerializeField] 
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 2000)]
    private float breakDepleteRate = 800f;
    
    [Header("Input")]

    // TODO make interface that inputs share
    [SerializeField] private InputManager _input;

    [Header("Dev")]
    [SerializeField] private float devTargetAltitudeSpeed = 5f;
    
    // PID settings
    [SerializeField] private float pTerm = 0f;
    [SerializeField] private float iTerm = 0f;
    [SerializeField] private float dTerm = 0f;
    
    
    [SerializeField] private float _uprightStrength = 10f;
    [SerializeField] private float _uprightStrengthDamper = 0.5f;

    
    // ########################################

    private Vector2 move;
    private float altitude;
    private float sails;
    private float lateralBrake;

    private Rigidbody rb;

    public Vector3 targetAltitude { get; private set; }

    private PIDController _pid = new();

    private void Start() {
        rb = GetComponent<Rigidbody>();
        targetAltitude = transform.position;
        
        Cursor.lockState = CursorLockMode.Locked;
        
        brakeForce = maxBrakeForce;
    }

    void Update() {

        updatePIDSettings();
        
        move = _input.move;
        altitude = _input.altitude;
        sails = _input.sails;
        lateralBrake = _input.lateralBrake;
        
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
    }
    
    private void UpdateLateralBreak() {
        depleteBreakGuage();
        
        if (lateralBrake > 0) {
            //Apply force until sideways velocity is 0 but retain forward and backward velocity
            
            rb.AddForce(-transform.right * lateralBrake * brakeForce, ForceMode.Force);
        } else if (lateralBrake < 0) {
            rb.AddForce(transform.right * -lateralBrake * brakeForce, ForceMode.Force);
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
        // TODO add pid controller to set target to rotate to
        // Multiplying by 40 as a hack because I thought turn was too low. //TODO make setting

        if (sails != 0) {
            rb.AddTorque(Vector3.up * (sails * turnRate * Time.deltaTime), ForceMode.Force);

        } else {
            
            // When no input is read on sails i.e. "0" the ship will slowly stop turning
            rb.AddTorque(-rb.angularVelocity * turnDrag, ForceMode.Force);
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
        float targetFloatAboveGround = 2f;
        if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.down), out hit, Mathf.Infinity)) {
            targetAltitude = new Vector3(0, (transform.position.y - hit.distance) + targetFloatAboveGround, 0);
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
