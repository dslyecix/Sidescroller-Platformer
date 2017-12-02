using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.BasicMovement
{
    public class MyCharacterController : BaseCharacterController
    {
        [Header("Stable Movement")]
        public float MaxStableMoveSpeed = 10f;
        public float StableMovementSharpness = 15;
        public float OrientationSharpness = 10;

        [Header("Air Movement")]
        public float MaxAirMoveSpeed = 10f;
        public float AirAccelerationSpeed = 1f;
        public float Drag = 0.1f;

        [Header("Misc")]
        public Vector3 Gravity = new Vector3(0, -30f, 0);

        private Vector3 _worldspaceMoveInputVector = Vector3.zero;
        private Vector3 _targetLookDirection = Vector3.zero;
        private Vector3 _smoothedLookDirection = Vector3.zero;

        /// <summary>
        /// This is called every frame by MyPlayer in order to tell the character where to go and where to look
        /// </summary>
        public void SetInputs(Vector3 cameraOrientedInputVector, Vector3 targetLookDirection)
        {
            // The worldspaceMoveInputVector represents the move input projected on the character plane
            _worldspaceMoveInputVector = Vector3.ProjectOnPlane(cameraOrientedInputVector, KinematicCharacterMotor.CharacterUp).normalized * cameraOrientedInputVector.magnitude;

            // The targetLookDirection is the direction we want the character to orient itself towards
            _targetLookDirection = Vector3.ProjectOnPlane(targetLookDirection, KinematicCharacterMotor.CharacterUp).normalized;
        }

        public override void BeforeCharacterUpdate(float deltaTime)
        {
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its rotation should be right now
        /// </summary>
        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            if (_targetLookDirection != Vector3.zero && OrientationSharpness > 0f)
            {
                // Smoothly interpolate from current to target look direction
                _smoothedLookDirection = Vector3.Lerp(_smoothedLookDirection, _targetLookDirection, 1f - Mathf.Exp(-OrientationSharpness * deltaTime)).normalized;

                // Set the current rotation (which will be used by the KinematicCharacterMotor)
                currentRotation = Quaternion.LookRotation(_smoothedLookDirection, KinematicCharacterMotor.CharacterUp);
            }
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its velocity should be right now
        /// </summary>
        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            Vector3 targetMovementVelocity = Vector3.zero;

            // Ground movement
            if (KinematicCharacterMotor.IsStableOnGround)
            {
                // Reorient current velocity on the ground slope before smoothing (important to avoid velocity loss in slope changes)
                currentVelocity = KinematicCharacterMotor.GetDirectionTangentToSurface(currentVelocity, KinematicCharacterMotor.GroundNormal) * currentVelocity.magnitude;

                // Calculate target velocity (still oriented on ground slope)
                Vector3 inputRight = Vector3.Cross(_worldspaceMoveInputVector, KinematicCharacterMotor.CharacterUp);
                Vector3 reorientedInput = Vector3.Cross(KinematicCharacterMotor.GroundNormal, inputRight).normalized * _worldspaceMoveInputVector.magnitude;
                targetMovementVelocity = reorientedInput * MaxStableMoveSpeed;

                // Smoothly interpolate to target velocity
                currentVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-StableMovementSharpness * deltaTime));
            }
            // Air movement
            else
            {
                if (_worldspaceMoveInputVector.sqrMagnitude > 0f)
                {
                    // If we want to move, add an acceleration to the velocity
                    targetMovementVelocity = _worldspaceMoveInputVector * MaxAirMoveSpeed;
                    Vector3 velocityDiff = Vector3.ProjectOnPlane(targetMovementVelocity - currentVelocity, Gravity);
                    currentVelocity += velocityDiff * AirAccelerationSpeed * deltaTime;
                }

                // Gravity
                currentVelocity += Gravity * deltaTime;

                // Drag
                currentVelocity *= (1f / (1f + (Drag * deltaTime)));
            }
        }

        public override void AfterCharacterUpdate(float deltaTime)
        {
        }

        public override bool CanBeStableOnCollider(Collider coll)
        {
            return true;
        }

        public override bool IsColliderValidForCollisions(Collider coll)
        {
            return true;
        }

        public override bool MustUpdateGrounding()
        {
            return true;
        }

        public override void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
        }

        public override void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
        }
    }
}