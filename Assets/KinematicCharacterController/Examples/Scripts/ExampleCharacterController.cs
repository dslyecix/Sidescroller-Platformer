﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;

public class ExampleCharacterController : BaseCharacterController
{        
    [Header("Stable Movement")]
    public float MaxStableMoveSpeed = 10f; // Max speed when stable on ground
    public float StableMovementSharpness = 15; // Sharpness of the acceleration when stable on ground
    public float OrientationSharpness = 10; // Sharpness of rotations when stable on ground

    [Header("Air Movement")]
    public float MaxAirMoveSpeed = 10f; // Max speed for air movement
    public float AirAccelerationSpeed = 5f; // Acceleration when in air
    public float Drag = 0.1f; // Air drag

    [Header("Jumping")]
    public bool AllowJumpingWhenSliding = false; // Is jumping allowed when we are sliding down a surface, even if we are not "stable" on it?
    public float JumpSpeed = 10f; // Strength of the jump impulse
    public float JumpPreGroundingGraceTime = 0f; // Time before landing that jump inputs will be remembered and applied at the moment of landing
    public float JumpPostGroundingGraceTime = 0f; // Time after getting un-grounded that jumping will still be allowed

    [Header("Misc")]
    public bool OrientTowardsGravity = true; // Should the character align its up direction with the gravity (used for the planet example)
    public Vector3 Gravity = new Vector3(0, -9.81f, 0); // Gravity vector
    public Transform MeshRoot; // This is the transform that will be scaled down while crouching

    [HideInInspector]
    public Collider[] IgnoredColliders = new Collider[] { };

    private Collider[] _probedColliders = new Collider[8];
    private Vector3 _moveInputVector = Vector3.zero;
    private Vector3 _lookInputVector = Vector3.zero;
    private Vector3 _smoothedLookInputDirection = Vector3.zero;
    private Vector3 _internalVelocityAdd = Vector3.zero;
    private bool _jumpRequested = false;
    private bool _jumpConsumed = false;
    private bool _jumpedThisFrame = false;
    private Vector3 _jumpDirection = Vector3.up;
    private float _timeSinceJumpRequested = Mathf.Infinity;
    private float _timeSinceLastAbleToJump = 0f;
    private bool _isTryingToUncrouch = false;

    /// <summary>
    /// This is called by the ExamplePlayer or the ExampleAIController to set the character's movement and look input vectors
    /// </summary>
    public void SetInputs(Vector3 moveInput, Vector3 lookInput)
    {
        _moveInputVector = Vector3.ProjectOnPlane(moveInput, KinematicCharacterMotor.CharacterUp).normalized * moveInput.magnitude;
        _lookInputVector = Vector3.ProjectOnPlane(lookInput, KinematicCharacterMotor.CharacterUp).normalized;
    }

    public override void BeforeCharacterUpdate(float deltaTime)
    {
    }

    public override bool MustUpdateGrounding()
    {
        // In this case, we always want to probe for ground. However, if we wanted to add a swimming 
        // movement mode for example, we wouldn't want to probe and snap to ground while we are swimming. In 
        // that case we would return false here
        return true;
    }

    public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
    {
        if (_lookInputVector != Vector3.zero && OrientationSharpness > 0f)
        {
            _smoothedLookInputDirection = Vector3.Slerp(_smoothedLookInputDirection, _lookInputVector, 1 - Mathf.Exp(-OrientationSharpness * deltaTime)).normalized;
            currentRotation = Quaternion.LookRotation(_smoothedLookInputDirection, KinematicCharacterMotor.CharacterUp);
        }
        if (OrientTowardsGravity)
        {
            currentRotation = Quaternion.FromToRotation((currentRotation * Vector3.up), -Gravity) * currentRotation;
        }
    }

    public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
    {
        Vector3 targetMovementVelocity = Vector3.zero;
        if (KinematicCharacterMotor.IsStableOnGround)
        {
            // Reorient velocity on slope
            currentVelocity = KinematicCharacterMotor.GetDirectionTangentToSurface(currentVelocity, KinematicCharacterMotor.GroundNormal) * currentVelocity.magnitude;

            // Calculate target velocity
            Vector3 inputRight = Vector3.Cross(_moveInputVector, KinematicCharacterMotor.CharacterUp);
            Vector3 reorientedInput = Vector3.Cross(KinematicCharacterMotor.GroundNormal, inputRight).normalized * _moveInputVector.magnitude;
            targetMovementVelocity = reorientedInput * MaxStableMoveSpeed;

            // Independant movement Velocity
            currentVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-StableMovementSharpness * deltaTime));
        }
        else
        {
            // Add move input
            if (_moveInputVector.sqrMagnitude > 0f)
            {
                targetMovementVelocity = _moveInputVector * MaxAirMoveSpeed;
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
            // See if we actually are allowed to jump
            if (!_jumpConsumed && ((AllowJumpingWhenSliding ? KinematicCharacterMotor.FoundAnyGround : KinematicCharacterMotor.IsStableOnGround) || (JumpPostGroundingGraceTime > 0 && _timeSinceLastAbleToJump <= JumpPostGroundingGraceTime)))
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

        // Grounding considerations
        if (KinematicCharacterMotor.IsStableOnGround && !KinematicCharacterMotor.WasStableOnGround)
        {
            OnLanded();
        }
        else if (!KinematicCharacterMotor.IsStableOnGround && KinematicCharacterMotor.WasStableOnGround)
        {
            OnLeaveStableGround();
        }

        if (AllowJumpingWhenSliding ? KinematicCharacterMotor.FoundAnyGround : KinematicCharacterMotor.IsStableOnGround)
        {
            _jumpConsumed = false;
            _timeSinceLastAbleToJump = 0f;
        }
        else
        {
            _timeSinceLastAbleToJump += deltaTime;
        }

        // Handle uncrouching
        if(_isTryingToUncrouch)
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

                _isTryingToUncrouch = false;
            }
        }
    }

    public override bool IsColliderValidForCollisions(Collider coll)
    {
        // Example of ignoring collisions with specific colliders
        for(int i = 0; i < IgnoredColliders.Length; i++)
        {
            if(coll == IgnoredColliders[i])
            {
                return false;
            }
        }

        return true;
    }

    public override bool CanBeStableOnCollider(Collider coll)
    {
        return true;
    }

    public override void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
    {
    }

    public override void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
    {
    }

    public void AddVelocity(Vector3 velocity)
    {
        _internalVelocityAdd += velocity;
    }

    public void Jump()
    {
        _timeSinceJumpRequested = 0f;
        _jumpRequested = true;
    }

    public void Crouch(bool crouch)
    {
        if(crouch)
        {
            KinematicCharacterMotor.SetCapsuleDimensionsAuto(0.5f, 1f);
            MeshRoot.localScale = new Vector3(1f, 0.5f, 1f);
        }
        else
        {
            _isTryingToUncrouch = true;
        }
    }

    protected void OnLanded()
    {
    }

    protected void OnLeaveStableGround()
    {
    }
}
