using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Hands;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float strafeSpeed = 2f;
    public float forwardSpeed = 2f;
    public float ascendSpeed = 1.5f;
    public float rotateSpeed = 80f;

    [Header("Hand Tracking Settings")]
    public float pinchThreshold = 0.03f;
    public float pinchHysteresis = 0.01f;
    public float pinchYawRange = 45f;
    public float handRotateSensitivity = 1f;
    public float fistThreshold = 0.04f;

    [Header("Hand/Controller Switching")]
    public float handIdleDelay = 0.5f; // seconds before switching to controllers

    [Header("Debug")]
    public bool debugLogs = false;

    private XRHandSubsystem handSubsystem;
    private Rigidbody rb;
    private float handIdleTimer = 0f;

    private bool isPinchingActive = false;
    private float pinchStartYaw = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
#if UNITY_6000_0_OR_NEWER
        rb.linearDamping = 2f;
        rb.angularDamping = 2f;
#endif

        var subsystems = new List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);
        if (subsystems.Count > 0)
        {
            handSubsystem = subsystems[0];
            if (debugLogs) Debug.Log("✅ XRHandSubsystem found: " + handSubsystem);
        }
        else
        {
            Debug.LogWarning("❌ No XRHandSubsystem found! Enable XR Hands + OpenXR Hand Tracking.");
        }
    }

    void Update()
    {
        bool handInputDetected = false;

        // ------------------ HAND INPUT DETECTION ------------------
        if (handSubsystem != null && handSubsystem.running)
        {
            bool leftTracked = handSubsystem.leftHand.isTracked;
            bool rightTracked = handSubsystem.rightHand.isTracked;

            if (leftTracked || rightTracked)
                handIdleTimer = 0f;

            float forwardInput = 0f, strafeInput = 0f, rotateInput = 0f;
            bool isAscend = false, isDescend = false, isFist = false;

            if (leftTracked || rightTracked)
            {
                GetHandInput(ref forwardInput, ref strafeInput, ref rotateInput, ref isAscend, ref isDescend, ref isFist);

                if (Mathf.Abs(forwardInput) > 0.01f ||
                    Mathf.Abs(strafeInput) > 0.01f ||
                    Mathf.Abs(rotateInput) > 0.01f ||
                    isAscend || isDescend || isFist)
                {
                    handInputDetected = true;
                    handIdleTimer = 0f;
                    ApplyDronePhysics(isAscend, isDescend, isFist, rotateInput, strafeInput, forwardInput);
                }
            }
        }

        // ------------------ HAND IDLE / CONTROLLER SWITCH ------------------
        if (!handInputDetected)
        {
            handIdleTimer += Time.deltaTime;

            if (handIdleTimer >= handIdleDelay &&
                (handSubsystem == null ||
                 (!handSubsystem.leftHand.isTracked && !handSubsystem.rightHand.isTracked)))
            {
                HandleControllerInput();
            }
            else
            {
                Hover();
            }
        }

        if (debugLogs && handSubsystem != null)
        {
            Debug.Log($"LeftTracked: {handSubsystem.leftHand.isTracked}, RightTracked: {handSubsystem.rightHand.isTracked}");
        }
    }

    // ------------------ HAND INPUT ------------------
    // ---------- GetHandInput (robust, player-relative) ----------
void GetHandInput(ref float forwardInput, ref float strafeInput, ref float rotateInput,
                  ref bool isAscend, ref bool isDescend, ref bool isFist)
{
    forwardInput = 0f;
    strafeInput = 0f;
    rotateInput = 0f;
    isAscend = false;
    isDescend = false;
    isFist = false;

    if (handSubsystem == null) return;

    Transform head = Camera.main != null ? Camera.main.transform : null;
    if (head == null)
    {
        if (debugLogs) Debug.LogWarning("Camera.main not found — using drone transform as reference");
        head = transform;
    }

    // --- RIGHT HAND: movement & gestures ---
    var right = handSubsystem.rightHand;
    if (right.isTracked)
    {
        var palm = right.GetJoint(XRHandJointID.Palm);
        var wrist = right.GetJoint(XRHandJointID.Wrist);
        var thumbTip = right.GetJoint(XRHandJointID.ThumbTip);
        var indexTip = right.GetJoint(XRHandJointID.IndexTip);
        var indexTipJ = right.GetJoint(XRHandJointID.IndexTip);
        var middleTip = right.GetJoint(XRHandJointID.MiddleTip);
        var ringTip = right.GetJoint(XRHandJointID.RingTip);
        var littleTip = right.GetJoint(XRHandJointID.LittleTip);

        // Fist detection (average distance of tips to palm)
        if (palm.TryGetPose(out Pose palmPose) &&
            indexTip.TryGetPose(out Pose iPose) &&
            middleTip.TryGetPose(out Pose mPose) &&
            ringTip.TryGetPose(out Pose rPose) &&
            littleTip.TryGetPose(out Pose lPose))
        {
            float avgDist =
                (Vector3.Distance(iPose.position, palmPose.position) +
                 Vector3.Distance(mPose.position, palmPose.position) +
                 Vector3.Distance(rPose.position, palmPose.position) +
                 Vector3.Distance(lPose.position, palmPose.position)) / 4f;

            if (avgDist < fistThreshold)
            {
                isFist = true;
                if (debugLogs) Debug.Log($"Fist detected (avgDist={avgDist:F3})");
                return; // immediate stop - no other inputs
            }
        }

        // PINCH detection for right-hand usage (if you prefer right pinch to rotate)
        bool rightPinch = false;
        if (thumbTip.TryGetPose(out Pose tPose) && indexTip.TryGetPose(out Pose idxPose))
        {
            float pinchDist = Vector3.Distance(tPose.position, idxPose.position);
            rightPinch = pinchDist < pinchThreshold;
            if (debugLogs) Debug.Log($"Right pinchDist={pinchDist:F3} pinch={rightPinch}");
        }

        // Movement: prefer palm pose for direction, fallback to wrist pos for strafe
        if (palm.TryGetPose(out Pose palmPoseforThumb))
{
    // compute hand position relative to head
    Vector3 handOffset = palmPose.position - head.position;

// Forward/back
float forwardDist = Vector3.Dot(handOffset, head.forward);
forwardInput = Mathf.Abs(forwardDist) > 0.15f ? Mathf.Clamp(forwardDist * 1.5f, -1f, 1f) : 0f;

// Strafe
float lateralDist = Vector3.Dot(handOffset, head.right);
strafeInput = Mathf.Abs(lateralDist) > 0.15f ? Mathf.Clamp(lateralDist * 1.5f, -1f, 1f) : 0f;

// Vertical
float verticalDist = handOffset.y;
if (verticalDist > 0.20f) isAscend = true;
else if (verticalDist < -0.15f) isDescend = true;


    // --- ✋ THUMB GESTURE DETECTION ---
    var thumb = right.GetJoint(XRHandJointID.ThumbTip);
    var indexBase = right.GetJoint(XRHandJointID.IndexMetacarpal);

    if (thumb.TryGetPose(out Pose thumbPose) && indexBase.TryGetPose(out Pose indexBasePose))
    {
        Vector3 thumbDir = (thumbPose.position - palmPoseforThumb.position).normalized;     // thumb direction
        Vector3 palmUp = palmPoseforThumb.up.normalized;

        float thumbDot = Vector3.Dot(thumbDir, Vector3.up);  // Compare thumb direction to world up/down

        if (thumbDot > 0.5f)
        {
            isAscend = true; // 👍 Thumbs up
            if (debugLogs) Debug.Log("🟢 Thumbs Up → Ascend");
        }
        else if (thumbDot < -0.5f)
        {
            isDescend = true; // 👎 Thumbs down
            if (debugLogs) Debug.Log("🔵 Thumbs Down → Descend");
        }
    }

    if (debugLogs)
    Debug.Log($"Right palm headOffset={handOffset:F3} forwardInput={forwardInput:F2} strafeInput={strafeInput:F2} ascend={isAscend} descend={isDescend}");

}

    }

    // --- LEFT HAND: rotation via pinch & yaw ---
    var left = handSubsystem.leftHand;
    if (left.isTracked)
    {
        var thumb = left.GetJoint(XRHandJointID.ThumbTip);
        var idx = left.GetJoint(XRHandJointID.IndexTip);
        var palmL = left.GetJoint(XRHandJointID.Palm);

        if (thumb.TryGetPose(out Pose tP) && idx.TryGetPose(out Pose iP))
        {
            float pinchDist = Vector3.Distance(tP.position, iP.position);
            if (!isPinchingActive && pinchDist < pinchThreshold)
            {
                isPinchingActive = true;
                // store starting yaw of camera (prefer head yaw)
                pinchStartYaw = head != null ? head.eulerAngles.y : transform.eulerAngles.y;
                if (debugLogs) Debug.Log($"Pinch start (dist={pinchDist:F3}) startYaw={pinchStartYaw:F2}");
            }
            else if (isPinchingActive && pinchDist > pinchThreshold + pinchHysteresis)
            {
                isPinchingActive = false;
                if (debugLogs) Debug.Log($"Pinch end (dist={pinchDist:F3})");
            }

            if (isPinchingActive && palmL.TryGetPose(out Pose palmLPose))
            {
                float currentYaw = head != null ? head.eulerAngles.y : transform.eulerAngles.y;
                float deltaYaw = Mathf.DeltaAngle(pinchStartYaw, currentYaw); // degrees
                rotateInput = Mathf.Clamp(deltaYaw / pinchYawRange, -1f, 1f);
                if (debugLogs) Debug.Log($"Pinch rotate deltaYaw={deltaYaw:F2} rotateInput={rotateInput:F2}");
            }
        }
    }
}



    // ------------------ CONTROLLER INPUT ------------------
    void HandleControllerInput()
    {
        float forwardInput = 0f;
        float strafeInput = 0f;
        float rotateInput = 0f;
        bool isAscend = false;
        bool isDescend = false;
        bool isFist = false;

        // RIGHT JOYSTICK -> movement
        InputDevice rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        if (rightDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightStick))
        {
            forwardInput = rightStick.y;   // forward/back
            strafeInput = rightStick.x;    // strafe
        }

        // LEFT JOYSTICK -> rotation
        InputDevice leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        if (leftDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftStick))
        {
            rotateInput = leftStick.x;     // yaw rotation
        }

        // Triggers -> ascend/descend
        if (rightDevice.TryGetFeatureValue(CommonUsages.trigger, out float rightTrigger) && rightTrigger > 0.05f)
            isAscend = true;
        if (leftDevice.TryGetFeatureValue(CommonUsages.trigger, out float leftTrigger) && leftTrigger > 0.05f)
            isDescend = true;

        ApplyDronePhysics(isAscend, isDescend, isFist, rotateInput, strafeInput, forwardInput);
    }

    // ------------------ PHYSICS ------------------
   // ---------- ApplyDronePhysics (use rb.velocity consistently) ----------
void ApplyDronePhysics(bool isAscend, bool isDescend, bool isFist,
                       float rotateInput, float strafeInput, float forwardInput)
{
    if (isFist)
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        return;
    }

    // Vertical control
    float vertical = 0f;
    if (isAscend) vertical = ascendSpeed;
    else if (isDescend) vertical = -ascendSpeed;

    // Compute movement
    Vector3 move =
        transform.forward * (forwardInput * forwardSpeed) +
        transform.right * (strafeInput * strafeSpeed) +
        Vector3.up * vertical;

    // Apply velocity directly — hover when idle
    if (move.magnitude > 0.01f)
        rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, move, Time.deltaTime * 4f);
    else
        Hover();

    // Rotation (yaw)
    if (Mathf.Abs(rotateInput) > 0.05f)
    {
        rb.AddTorque(Vector3.up * rotateInput * rotateSpeed * Mathf.Deg2Rad, ForceMode.Acceleration);
    }
}
void Hover()
{
    rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, Vector3.zero, Time.deltaTime * 2f);
    rb.angularVelocity = Vector3.zero;
}

}



