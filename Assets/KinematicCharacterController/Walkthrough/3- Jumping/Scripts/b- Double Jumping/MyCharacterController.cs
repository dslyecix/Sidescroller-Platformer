using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.DoubleJumping
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

        [Header("Jumping")]
        public bool AllowJumpingWhenSliding = false;
        public bool AllowDoubleJump = false;
        public float JumpSpeed = 10f;
        public float JumpPreGroundingGraceTime = 0f;
        public float JumpPostGroundingGraceTime = 0f;

        [Header("Misc")]
        public Vector3 Gravity = new Vector3(0, -30f, 0);

        private Vector3 _worldspaceMoveInputVector = Vector3.zero;
        private Vector3 _targetLookDirection = Vector3.zero;
        private Vector3 _smoothedLookDirection = Vector3.zero;
        private bool _jumpRequested = false;
        private bool _jumpConsumed = false;
        private bool _doubleJumpConsumed = false;
        private bool _jumpedThisFrame = false;
        private float _timeSinceJumpRequested = Mathf.Infinity;
        private float _timeSinceLastAbleToJump = 0f;

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
        /// This is where you tell your character what its rotation should be right now. 
        /// This is the ONLY place where you should set the character's rotation
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
        /// This is where you tell your character what its velocity should be right now. 
        /// This is the ONLY place where you can set the character's velocity
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

            // Handle jumping
            _jumpedThisFrame = false;
            _timeSinceJumpRequested += deltaTime;
            if (_jumpRequested)
            {
                // Handle double jump
                if (AllowDoubleJump)
                {
                    if (_jumpConsumed && !_doubleJumpConsumed && (AllowJumpingWhenSliding ? !KinematicCharacterMotor.FoundAnyGround : !KinematicCharacterMotor.IsStableOnGround))
                    {
                        KinematicCharacterMotor.ForceUnground();

                        // Add to the return velocity and reset jump state
                        currentVelocity += (KinematicCharacterMotor.CharacterUp * JumpSpeed) - Vector3.Project(currentVelocity, KinematicCharacterMotor.CharacterUp);
                        _jumpRequested = false;
                        _doubleJumpConsumed = true;
                        _jumpedThisFrame = true;
                    }
                }

                // See if we actually are allowed to jump
                if (!_jumpConsumed && ((AllowJumpingWhenSliding ? KinematicCharacterMotor.FoundAnyGround : KinematicCharacterMotor.IsStableOnGround) || _timeSinceLastAbleToJump <= JumpPostGroundingGraceTime))
                {
                    // Calculate jump direction before ungrounding
                    Vector3 jumpDirection = KinematicCharacterMotor.CharacterUp;
                    if (KinematicCharacterMotor.FoundAnyGround && !KinematicCharacterMotor.IsStableOnGround)
                    {
                        jumpDirection = KinematicCharacterMotor.GroundNormal;
                    }

                    // Makes the character skip ground probing/snapping on its next update. 
                    // If this line weren't here, the character would remain snapped to the ground when trying to jump. Try commenting this line out and see.
                    KinematicCharacterMotor.ForceUnground();

                    // Add to the return velocity and reset jump state
                    currentVelocity += (jumpDirection * JumpSpeed) - Vector3.Project(currentVelocity, KinematicCharacterMotor.CharacterUp);
                    _jumpRequested = false;
                    _jumpConsumed = true;
                    _jumpedThisFrame = true;
                }
            }
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called after the character has finished its movement update
        /// </summary>
        public override void AfterCharacterUpdate(float deltaTime)
        {
            // Handle jump-related values
            {
                // Handle jumping pre-ground grace period
                if (_jumpRequested && _timeSinceJumpRequested > JumpPreGroundingGraceTime)
                {
                    _jumpRequested = false;
                }

                if (AllowJumpingWhenSliding ? KinematicCharacterMotor.FoundAnyGround : KinematicCharacterMotor.IsStableOnGround)
                {
                    // If we're on a ground surface, reset jumping values
                    if (!_jumpedThisFrame)
                    {
                        _doubleJumpConsumed = false;
                        _jumpConsumed = false;
                    }
                    _timeSinceLastAbleToJump = 0f;
                }
                else
                {
                    // Keep track of time since we were last able to jump (for grace period)
                    _timeSinceLastAbleToJump += deltaTime;
                }
            }
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

        /// <summary>
        /// This is called by MyPlayer upon jump input press
        /// </summary>
        public void OnJump()
        {
            // Init jumping values
            _timeSinceJumpRequested = 0f;
            _jumpRequested = true;
        }
    }
}