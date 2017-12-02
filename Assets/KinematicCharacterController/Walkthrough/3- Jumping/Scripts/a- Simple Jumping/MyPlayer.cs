using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using KinematicCharacterController.Examples;

namespace KinematicCharacterController.Walkthrough.SinpleJumping
{
    public class MyPlayer : MonoBehaviour
    {
        public OrbitCamera OrbitCamera;
        public Transform CameraFollowPoint;
        public MyCharacterController Character;
        public float MouseSensitivity = 0.01f;

        private Vector3 _lookInputVector = Vector3.zero;
        private Vector3 _moveInputVector = Vector3.zero;

        private void Start()
        {
            Cursor.lockState = CursorLockMode.Locked;

            // Tell camera to follow transform
            OrbitCamera.SetFollowTransform(CameraFollowPoint);

            // Ignore the character's collider(s) for camera obstruction checks
            OrbitCamera.IgnoredColliders = Character.GetComponentsInChildren<Collider>();
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                Cursor.lockState = CursorLockMode.Locked;
            }

            HandleCameraInput();
            HandleCharacterInput();
        }

        private void HandleCameraInput()
        {
            // Create the look input vector for the camera
            float mouseLookAxisUp = Input.GetAxisRaw("Mouse Y");
            float mouseLookAxisRight = Input.GetAxisRaw("Mouse X");
            _lookInputVector = new Vector3(mouseLookAxisRight * MouseSensitivity, mouseLookAxisUp * MouseSensitivity, 0f);

            // Prevent moving the camera while the cursor isn't locked
            if (Cursor.lockState != CursorLockMode.Locked)
            {
                _lookInputVector = Vector3.zero;
            }

            // Input for zooming the camera (disabled in WebGL because it can cause problems)
            float scrollInput = -Input.GetAxis("Mouse ScrollWheel");
#if UNITY_WEBGL
        scrollInput = 0f;
#endif

            // Apply inputs to the camera
            OrbitCamera.SetInputs(scrollInput, _lookInputVector);

            // Handle toggling zoom level
            if (Input.GetMouseButtonDown(1))
            {
                OrbitCamera.TargetDistance = (OrbitCamera.TargetDistance == 0f) ? OrbitCamera.DefaultDistance : 0f;
            }
        }

        private void HandleCharacterInput()
        {
            // Create the move input vector for the character
            float moveAxisForward = Input.GetAxisRaw("Vertical");
            float moveAxisRight = Input.GetAxisRaw("Horizontal");
            _moveInputVector = new Vector3(moveAxisRight, 0f, moveAxisForward);
            _moveInputVector = Vector3.ClampMagnitude(_moveInputVector, 1f);

            // Convert this move input vector to the camera view space, so that pressing forward moves in the direction that the camera is pointing to
            Vector3 cameraOrientedInput = Quaternion.LookRotation(OrbitCamera.Transform.forward, OrbitCamera.Transform.up) * _moveInputVector;

            // Apply move input to character
            Character.SetInputs(cameraOrientedInput, OrbitCamera.PlanarDirection);

            // Jump input
            if (Input.GetKeyDown(KeyCode.Space))
            {
                Character.OnJump();
            }
        }
    }
}