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
    public float fistThreshold = 0.04f;
    public float strafeSensitivity = 2f;
    public float forwardSensitivity = 2f;
    public float handRotateSensitivity = 1f;

    private XRHandSubsystem handSubsystem;
    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.linearDamping = 2f;
        rb.angularDamping = 2f;

        var subsystems = new List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);
        if (subsystems.Count > 0)
        {
            handSubsystem = subsystems[0];
            Debug.Log("✅ XRHandSubsystem found: " + handSubsystem);
        }
        else
        {
            Debug.LogError("❌ No XRHandSubsystem found! Enable XR Hands + OpenXR Hand Tracking.");
        }
    }

    void Update()
    {
        if (handSubsystem != null && handSubsystem.running && handSubsystem.rightHand.isTracked)
            HandleHandInput();
        else
            HandleControllerInput();
    }

    // ------------------ CONTROLLER INPUT ------------------
    void HandleControllerInput()
    {
        InputDevice leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        InputDevice rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        bool isAscend = false;
        bool isDescend = false;
        bool isFist = false;
        bool isPinching = false;
        float rotateInput = 0f;
        float strafeInput = 0f;
        float forwardInput = 0f;

        // --- Left joystick →  = strafe, forward/back
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftStick))
        {
            forwardInput = leftStick.y;
            strafeInput = leftStick.x;
        }

        // --- Right joystick → rotation
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightStick))
        {
            if (Mathf.Abs(rightStick.x) > 0.1f)
            {
                rotateInput = rightStick.x;
                isPinching = true; // flag to apply rotation
            }
        }

        // --- Left trigger → ascend
        if (leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTrigger) && leftTrigger)
            isAscend = true;

        // --- Right trigger → descend
        if (rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTrigger) && rightTrigger)
            isDescend = true;

        ApplyDronePhysics(isAscend, isDescend, isFist, isPinching, rotateInput, strafeInput, forwardInput);
    }

    // ------------------ HAND INPUT ------------------
    void HandleHandInput()
    {
        var rightHand = handSubsystem.rightHand;

        bool isAscend = false;
        bool isDescend = false;
        bool isFist = false;
        bool isPinching = false;
        float rotateInput = 0f;
        float strafeInput = 0f;
        float forwardInput = 0f;

        var palm = rightHand.GetJoint(XRHandJointID.Palm);
        var wrist = rightHand.GetJoint(XRHandJointID.Wrist);
        var thumbTip = rightHand.GetJoint(XRHandJointID.ThumbTip);
        var indexTip = rightHand.GetJoint(XRHandJointID.IndexTip);
        var middleTip = rightHand.GetJoint(XRHandJointID.MiddleTip);
        var ringTip = rightHand.GetJoint(XRHandJointID.RingTip);
        var littleTip = rightHand.GetJoint(XRHandJointID.LittleTip);

        if (palm.TryGetPose(out Pose palmPose))
        {
            Vector3 palmUp = palmPose.up;
            Vector3 palmForward = palmPose.forward;

            if (Vector3.Dot(palmUp, Vector3.up) > 0.5f) isAscend = true;
            else if (Vector3.Dot(palmUp, Vector3.down) > 0.5f) isDescend = true;

            float forwardDot = Vector3.Dot(palmForward, Camera.main.transform.forward);
            if (forwardDot > 0.5f) forwardInput = 1f;
            else if (forwardDot < -0.5f) forwardInput = -1f;

            if (thumbTip.TryGetPose(out Pose tPose) && indexTip.TryGetPose(out Pose iPose))
            {
                float pinchDist = Vector3.Distance(tPose.position, iPose.position);
                if (pinchDist < pinchThreshold)
                {
                    isPinching = true;
                    float yaw = palmPose.rotation.eulerAngles.y;
                    float relativeYaw = Mathf.DeltaAngle(Camera.main.transform.eulerAngles.y, yaw);
                    rotateInput = Mathf.Clamp(relativeYaw / 45f, -1f, 1f) * handRotateSensitivity;
                }
            }

            if (thumbTip.TryGetPose(out Pose th) &&
                indexTip.TryGetPose(out Pose ix) &&
                middleTip.TryGetPose(out Pose mi) &&
                ringTip.TryGetPose(out Pose ri) &&
                littleTip.TryGetPose(out Pose li))
            {
                float avgDist =
                    (Vector3.Distance(th.position, palmPose.position) +
                     Vector3.Distance(ix.position, palmPose.position) +
                     Vector3.Distance(mi.position, palmPose.position) +
                     Vector3.Distance(ri.position, palmPose.position) +
                     Vector3.Distance(li.position, palmPose.position)) / 5f;

                if (avgDist < fistThreshold) isFist = true;
            }
        }

        if (wrist.TryGetPose(out Pose wristPose))
        {
            Vector3 headPos = Camera.main.transform.position;
            Vector3 handOffset = wristPose.position - headPos;
            strafeInput = Vector3.Dot(handOffset, Camera.main.transform.right);
        }

        ApplyDronePhysics(isAscend, isDescend, isFist, isPinching, rotateInput, strafeInput, forwardInput);
    }

    // ------------------ DRONE PHYSICS ------------------
    void ApplyDronePhysics(bool isAscend, bool isDescend, bool isFist, bool isPinching, float rotateInput, float strafeInput, float forwardInput)
    {
        if (isFist)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            return;
        }

        float vertical = 0f;
        if (isAscend) vertical = ascendSpeed;
        else if (isDescend) vertical = -ascendSpeed;

        float horizontal = Mathf.Clamp(strafeInput * strafeSpeed, -strafeSpeed, strafeSpeed);
        float forward = forwardInput * forwardSpeed;

        Vector3 move = Camera.main.transform.right * horizontal +
                       Camera.main.transform.forward * forward +
                       Vector3.up * vertical;

        rb.linearVelocity = move;

        if (isPinching && Mathf.Abs(rotateInput) > 0.1f)
            rb.AddTorque(Vector3.up * rotateInput * rotateSpeed * Time.deltaTime, ForceMode.VelocityChange);
        else
            rb.angularVelocity = Vector3.zero;
    }
}


