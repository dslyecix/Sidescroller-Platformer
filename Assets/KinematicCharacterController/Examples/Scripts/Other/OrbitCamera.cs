﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Examples
{
    public class OrbitCamera : MonoBehaviour
    {
        [Header("Framing")]
        public Camera Camera;
        public Vector2 FollowTransformFraming = new Vector2(0f, 0f);
        public float FollowingSharpness = 30f;

        [Header("Distance")]
        public float DefaultDistance = 6f;
        public float MinDistance = 2f;
        public float MaxDistance = 10f;
        public float DistanceMovementSpeed = 10f;
        public float DistanceMovementSharpness = 10f;

        [Header("Rotation")]
        public bool InvertX = false;
        public bool InvertY = false;
        [Range(-90f, 90f)]
        public float DefaultVerticalAngle = 20f;
        [Range(-90f, 90f)]
        public float MinVerticalAngle = -80f;
        [Range(-90f, 90f)]
        public float MaxVerticalAngle = 80f;
        public float RotationSpeed = 10f;
        public float RotationSharpness = 30f;

        [Header("Obstruction")]
        public float ObstructionCheckRadius = 0.5f;
        public LayerMask ObstructionLayers = -1;
        public float ObstructionSharpness = 10000f;

        public Transform Transform { get; private set; }
        public Vector3 PlanarDirection { get; private set; }
        public Transform FollowTransform { get; set; }
        public Collider[] IgnoredColliders { get; set; }
        public float TargetDistance { get; set; }

        private Vector3 _rotationInput;
        private bool _distanceIsObstructed;
        private float _zoomInput;
        private float _currentDistance;
        private float _targetVerticalAngle;
        private RaycastHit _obstructionHit;
        private int _obstructionCount;
        private RaycastHit[] _obstructions = new RaycastHit[MaxObstructions];
        private float _obstructionTime;
        private Vector3 _currentFollowPosition;

        private const int MaxObstructions = 32;

        void OnValidate()
        {
            DefaultDistance = Mathf.Clamp(DefaultDistance, MinDistance, MaxDistance);
            DefaultVerticalAngle = Mathf.Clamp(DefaultVerticalAngle, MinVerticalAngle, MaxVerticalAngle);
        }

        void Awake()
        {
            Transform = this.transform;

            _currentDistance = DefaultDistance;
            TargetDistance = _currentDistance;

            _targetVerticalAngle = 0f;

            PlanarDirection = Vector3.forward;
        }

        // Set the transform that the camera will orbit around
        public void SetFollowTransform(Transform followTransform)
        {
            FollowTransform = followTransform;
            PlanarDirection = followTransform.forward;
            _currentFollowPosition = FollowTransform.position;
        }

        // Receive input from the player
        public void SetInputs(float zoomInput, Vector3 rotationInput)
        {
            if (InvertX)
            {
                rotationInput.x *= -1f;
            }
            if (InvertY)
            {
                rotationInput.y *= -1f;
            }

            _rotationInput = rotationInput;
            _zoomInput = zoomInput;
        }

        private void LateUpdate()
        {
            float deltaTime = Time.deltaTime;
            
            if (FollowTransform)
            {
                // Process rotation input
                Quaternion rotationFromInput = Quaternion.Euler(FollowTransform.up * (_rotationInput.x * RotationSpeed));
                PlanarDirection = rotationFromInput * PlanarDirection;
                PlanarDirection = Vector3.Cross(FollowTransform.up, Vector3.Cross(PlanarDirection, FollowTransform.up));
                _targetVerticalAngle -= (_rotationInput.y * RotationSpeed);
                _targetVerticalAngle = Mathf.Clamp(_targetVerticalAngle, MinVerticalAngle, MaxVerticalAngle);

                // Process distance input
                if (_distanceIsObstructed && Mathf.Abs(_zoomInput) > 0f)
                {
                    TargetDistance = _currentDistance;
                }
                TargetDistance += _zoomInput * DistanceMovementSpeed;
                TargetDistance = Mathf.Clamp(TargetDistance, MinDistance, MaxDistance);

                // Find the smoothed follow position
                _currentFollowPosition = Vector3.Lerp(_currentFollowPosition, FollowTransform.position, 1f - Mathf.Exp(-FollowingSharpness * deltaTime));
                
                // Calculate smoothed rotation
                Quaternion planarRot = Quaternion.LookRotation(PlanarDirection, FollowTransform.up);
                Quaternion verticalRot = Quaternion.Euler(_targetVerticalAngle, 0, 0);
                Quaternion targetRotation = Quaternion.Slerp(Transform.rotation, planarRot * verticalRot, 1f - Mathf.Exp(-RotationSharpness * deltaTime));

                // Apply rotation
                Transform.rotation = targetRotation;

                // Handle obstructions
                {
                    RaycastHit closestHit = new RaycastHit();
                    closestHit.distance = Mathf.Infinity;
                    _obstructionCount = Physics.SphereCastNonAlloc(_currentFollowPosition, ObstructionCheckRadius, -Transform.forward, _obstructions, TargetDistance, ObstructionLayers, QueryTriggerInteraction.Ignore);
                    for (int i = 0; i < _obstructionCount; i++)
                    {
                        bool isIgnored = false;
                        for (int j = 0; j < IgnoredColliders.Length; j++)
                        {
                            if (IgnoredColliders[j] == _obstructions[i].collider)
                            {
                                isIgnored = true;
                                break;
                            }
                        }

                        if (!isIgnored && _obstructions[i].distance < closestHit.distance && _obstructions[i].distance > 0)
                        {
                            closestHit = _obstructions[i];
                        }
                    }

                    // If obstructions detecter
                    if (closestHit.distance < Mathf.Infinity)
                    {
                        _distanceIsObstructed = true;
                        _currentDistance = Mathf.Lerp(_currentDistance, closestHit.distance, 1 - Mathf.Exp(-ObstructionSharpness * deltaTime));
                    }
                    // If no obstruction
                    else
                    {
                        _distanceIsObstructed = false;
                        _currentDistance = Mathf.Lerp(_currentDistance, TargetDistance, 1 - Mathf.Exp(-DistanceMovementSharpness * deltaTime));
                    }
                }

                // Find the smoothed camera orbit position
                Vector3 targetPosition = _currentFollowPosition - ((targetRotation * Vector3.forward) * _currentDistance);

                // Handle framing
                targetPosition += Transform.right * FollowTransformFraming.x;
                targetPosition += Transform.up * FollowTransformFraming.y;

                // Apply position
                Transform.position = targetPosition;
            }
        }
    }
}