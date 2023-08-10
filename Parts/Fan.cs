using UnityEngine;

[CreateAssetMenu(fileName = "NewFan", menuName = "ScriptableObjects/CarParts/Fan", order = 3)]
public class Fan : ScriptableObject {
    
    [Tooltip("How fast the ship can change altitude, affected by hovercar mass. " +
             "This altitude is for over terrain, NOT the rest height speed.")]
    [Range(1, 200)]
    public float verticalAcceleration;
    
    [Tooltip("How fast the ship will return to the target altitude, based on" +
             "rest height.")]
    public float targetAltitudeSpeed;
    
    [Tooltip("How fast the ship will accelerate to the max speed.")]
    [Range(1, 100)]
    public float accelerationRate;

    [Tooltip("Maximum horizontal speed of hovercar")]
    [Range(1, 100)]
    public float maxSpeed;


}
