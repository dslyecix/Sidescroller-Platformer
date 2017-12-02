using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.ChargingState
{
    [System.Serializable]
    public class MyDefaultMovementState : MyMovementState
    {
        [Header("Stable Movement")]
        public float MaxStableMoveSpeed = 10f;
        public float StableMovementSharpness = 15;
        public float OrientationSharpness = 10;

        [Header("Air Movement")]
        public float MaxAirMoveSpeed = 10f;
        public float AirAccelerationSpeed = 5f;
        public float Drag = 0.1f;

        [Header("Jumping")]
        public bool AllowJumpingWhenSliding = false;
        public bool AllowDoubleJump = false;
        public bool AllowWallJump = false;
        public float JumpSpeed = 10f;
        public float JumpPreGroundingGraceTime = 0f;
        public float JumpPostGroundingGraceTime = 0f;

        [Header("Misc")]
        public List<Collider> IgnoredColliders = new List<Collider>();
        public bool OrientTowardsGravity = false;
        public Vector3 Gravity = new Vector3(0, -30f, 0);
        public Transform MeshRoot;

        public bool IsCrouching { get; private set; }

        private Collider[] _probedColliders = new Collider[8];
        private Vector3 _smoothedLookDirection = Vector3.zero;
        private bool _jumpRequested = false;
        private bool _jumpConsumed = false;
        private bool _doubleJumpConsumed = false;
        private bool _jumpedThisFrame = false;
        private bool _canWallJump = false;
        private Vector3 _wallJumpNormal;
        private float _timeSinceJumpRequested = Mathf.Infinity;
        private float _timeSinceLastAbleToJump = 0f;
        private Vector3 _internalVelocityAdd = Vector3.zero;
        private bool _shouldBeCrouching = false;
        
        public override void OnStateExit(MyMovementState nextState)
        {
        }

        public override void OnStateEnter(MyMovementState previousState)
        {
        }

        public override void BeforeCharacterUpdate(float deltaTime)
        {
        }

        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            if (AssignedCharacterController.TargetLookDirection != Vector3.zero && OrientationSharpness > 0f)
            {
                // Smoothly interpolate from current to target look direction
                _smoothedLookDirection = Vector3.Lerp(_smoothedLookDirection, AssignedCharacterController.TargetLookDirection, 1f - Mathf.Exp(-OrientationSharpness * deltaTime)).normalized;

                // Set the current rotation (which will be used by the KinematicCharacterMotor)
                currentRotation = Quaternion.LookRotation(_smoothedLookDirection, KinematicCharacterMotor.CharacterUp);
            }
            if (OrientTowardsGravity)
            {
                // Rotate from current up to invert gravity
                currentRotation = Quaternion.FromToRotation((currentRotation * Vector3.up), -Gravity) * currentRotation;
            }
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            Vector3 targetMovementVelocity = Vector3.zero;

            // Ground movement
            if (KinematicCharacterMotor.IsStableOnGround)
            {
                // Reorient current velocity on the ground slope before smoothing (important to avoid velocity loss in slope changes)
                currentVelocity = KinematicCharacterMotor.GetDirectionTangentToSurface(currentVelocity, KinematicCharacterMotor.GroundNormal) * currentVelocity.magnitude;

                // Calculate target velocity (still oriented on ground slope)
                Vector3 inputRight = Vector3.Cross(AssignedCharacterController.WorldspaceMoveInputVector, KinematicCharacterMotor.CharacterUp);
                Vector3 reorientedInput = Vector3.Cross(KinematicCharacterMotor.GroundNormal, inputRight).normalized * AssignedCharacterController.WorldspaceMoveInputVector.magnitude;
                targetMovementVelocity = reorientedInput * MaxStableMoveSpeed;

                // Smoothly interpolate to target velocity
                currentVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-StableMovementSharpness * deltaTime));
            }
            // Air movement
            else
            {
                if (AssignedCharacterController.WorldspaceMoveInputVector.sqrMagnitude > 0f)
                {
                    // If we want to move, add an acceleration to the velocity
                    targetMovementVelocity = AssignedCharacterController.WorldspaceMoveInputVector * MaxAirMoveSpeed;
                    Vector3 velocityDiff = Vector3.ProjectOnPlane(targetMovementVelocity - currentVelocity, Gravity);
                    currentVelocity += velocityDiff * AirAccelerationSpeed * deltaTime;
                }

                // Gravity
                currentVelocity += Gravity * deltaTime;

                // Drag
                currentVelocity *= (1f / (1f + (Drag * deltaTime)));
            }

            // Handle jumping
            {
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
                    if (_canWallJump ||
                        (!_jumpConsumed && ((AllowJumpingWhenSliding ? KinematicCharacterMotor.FoundAnyGround : KinematicCharacterMotor.IsStableOnGround) || _timeSinceLastAbleToJump <= JumpPostGroundingGraceTime)))
                    {
                        // Calculate jump direction before ungrounding
                        Vector3 jumpDirection = KinematicCharacterMotor.CharacterUp;
                        if (_canWallJump)
                        {
                            jumpDirection = _wallJumpNormal;
                        }
                        else if (KinematicCharacterMotor.FoundAnyGround && !KinematicCharacterMotor.IsStableOnGround)
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

                // Reset wall jump
                _canWallJump = false;
            }

            // Take into account additive velocity
            if (_internalVelocityAdd.sqrMagnitude > 0f)
            {
                currentVelocity += _internalVelocityAdd;
                _internalVelocityAdd = Vector3.zero;
            }
        }

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

            // Handle landing and leaving ground
            if (KinematicCharacterMotor.IsStableOnGround && !KinematicCharacterMotor.WasStableOnGround)
            {
                OnLanded();
            }
            else if (!KinematicCharacterMotor.IsStableOnGround && KinematicCharacterMotor.WasStableOnGround)
            {
                OnLeaveStableGround();
            }

            // Handle uncrouching
            if (IsCrouching && !_shouldBeCrouching)
            {
                // Do an overlap test with the character's standing height to see if there are any obstructions
                KinematicCharacterMotor.SetCapsuleDimensionsAuto(0.5f, 2f);
                if (KinematicCharacterMotor.CharacterOverlap(
                    KinematicCharacterMotor.TransientPosition,
                    KinematicCharacterMotor.TransientRotation,
                    _probedColliders,
                    KinematicCharacterMotor.CollidableLayers,
                    QueryTriggerInteraction.Ignore) > 0)
                {
                    // If obstructions, just stick to crouching dimensions
                    KinematicCharacterMotor.SetCapsuleDimensionsAuto(0.5f, 1f);
                }
                else
                {
                    // If no obstructions, uncrouch
                    MeshRoot.localScale = new Vector3(1f, 1f, 1f);
                    IsCrouching = false;
                }
            }
        }

        public override bool CanBeStableOnCollider(Collider coll)
        {
            return true;
        }

        public override bool IsColliderValidForCollisions(Collider coll)
        {
            if (IgnoredColliders.Contains(coll))
            {
                return false;
            }

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
            // We can wall jump only if we are not stable on ground and are moving against an obstruction
            if (AllowWallJump && !KinematicCharacterMotor.IsStableOnGround && !isStableOnHit)
            {
                _canWallJump = true;
                _wallJumpNormal = hitNormal;
            }
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

        /// <summary>
        /// This is called by MyPlayer upon crouch input press
        /// </summary>
        public void OnCrouch(bool crouch)
        {
            _shouldBeCrouching = crouch;

            if (crouch)
            {
                IsCrouching = true;

                KinematicCharacterMotor.SetCapsuleDimensionsAuto(0.5f, 1f);
                MeshRoot.localScale = new Vector3(1f, 0.5f, 1f);
            }
        }

        protected void OnLanded()
        {
        }

        protected void OnLeaveStableGround()
        {
        }

        public void AddVelocity(Vector3 velocity)
        {
            _internalVelocityAdd += velocity;
        }
    }
}