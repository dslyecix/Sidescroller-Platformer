using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.SwimmingState
{
    public class MyCharacterController : BaseCharacterController
    {
        [Header("Movement states")]
        public MyDefaultMovementState DefaultMovementState = new MyDefaultMovementState();
        public MyChargingState ChargingState = new MyChargingState();
        public MyNoClipState NoClipState = new MyNoClipState();
        public MySwimmingState SwimmingState = new MySwimmingState();

        [Header("Water detection")]
        public Transform SwimmingReferencePoint;
        public LayerMask WaterLayer;

        public Vector3 WorldspaceMoveInputVector { get; private set; }
        public Vector3 WorldspaceCharacterPlaneMoveInputVector { get; private set; }
        public Vector3 TargetLookDirection { get; private set; }
        public float VerticalInput { get; private set; }
        public MyMovementState CurrentMovementState { get; private set; }

        private Collider[] _waterOverlap = new Collider[1];

        private void Start()
        {
            // Handle initial state
            TransitionToState(DefaultMovementState);
        }

        /// <summary>
        /// Handles movement state transitions and enter/exit callbacks
        /// </summary>
        public void TransitionToState(MyMovementState newState)
        {
            newState.AssignedCharacterController = this;
            newState.KinematicCharacterMotor = this.KinematicCharacterMotor;

            if (CurrentMovementState != null)
            {
                CurrentMovementState.OnStateExit(newState);
            }

            MyMovementState prevState = CurrentMovementState;
            CurrentMovementState = newState;
            CurrentMovementState.OnStateEnter(prevState);
        }

        /// <summary>
        /// This is called every frame by MyPlayer in order to tell the character where to move to
        /// </summary>
        public void SetMoveVectorInput(Vector3 cameraOrientedInputVector)
        {
            WorldspaceMoveInputVector = cameraOrientedInputVector;

            // The WorldspaceCharacterPlaneMoveInputVector represents the move input projected on the character plane
            WorldspaceCharacterPlaneMoveInputVector = Vector3.ProjectOnPlane(cameraOrientedInputVector, KinematicCharacterMotor.CharacterUp).normalized * cameraOrientedInputVector.magnitude;
        }

        /// <summary>
        /// This is called every frame by MyPlayer in order to tell the character where to look
        /// </summary>
        public void SetLookDirectionInput(Vector3 targetLookDirection)
        {
            // The targetLookDirection is the direction we want the character to orient itself towards
            TargetLookDirection = Vector3.ProjectOnPlane(targetLookDirection, KinematicCharacterMotor.CharacterUp).normalized;
        }

        /// <summary>
        /// This is called every frame by MyPlayer in order to tell the character what its vertical input is (used by NoClip and Swimming states)
        /// </summary>
        public void SetVerticalInput(float verticalInput)
        {
            VerticalInput = verticalInput;
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called before the character begins its movement update
        /// </summary>
        public override void BeforeCharacterUpdate(float deltaTime)
        {
            // Handle detecting water surfaces
            {
                // Do a character overlap test to detect water surfaces
                if (KinematicCharacterMotor.CharacterOverlap(KinematicCharacterMotor.TransientPosition, KinematicCharacterMotor.TransientRotation, _waterOverlap, WaterLayer, QueryTriggerInteraction.Collide) > 0)
                {
                    // If a water surface was detected
                    if (_waterOverlap[0] != null)
                    {
                        // If the swimming reference point is inside the box, make sure we are in swimming state
                        if (Physics.ClosestPoint(SwimmingReferencePoint.position, _waterOverlap[0], _waterOverlap[0].transform.position, _waterOverlap[0].transform.rotation) == SwimmingReferencePoint.position)
                        {
                            if (CurrentMovementState == DefaultMovementState)
                            {
                                TransitionToState(SwimmingState);
                                SwimmingState.WaterZone = _waterOverlap[0];
                            }
                        }
                        // otherwise; default state
                        else
                        {
                            if (CurrentMovementState == SwimmingState)
                            {
                                TransitionToState(DefaultMovementState);
                            }
                        }
                    }
                }
            }

            CurrentMovementState.BeforeCharacterUpdate(deltaTime);
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its rotation should be right now. 
        /// This is the ONLY place where you should set the character's rotation
        /// </summary>
        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            CurrentMovementState.UpdateRotation(ref currentRotation, deltaTime);
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its velocity should be right now. 
        /// This is the ONLY place where you can set the character's velocity
        /// </summary>
        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            CurrentMovementState.UpdateVelocity(ref currentVelocity, deltaTime);
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called after the character has finished its movement update
        /// </summary>
        public override void AfterCharacterUpdate(float deltaTime)
        {
            CurrentMovementState.AfterCharacterUpdate(deltaTime);
        }

        public override bool CanBeStableOnCollider(Collider coll)
        {
            return CurrentMovementState.CanBeStableOnCollider(coll);
        }

        public override bool IsColliderValidForCollisions(Collider coll)
        {
            return CurrentMovementState.IsColliderValidForCollisions(coll);
        }

        public override bool MustUpdateGrounding()
        {
            return CurrentMovementState.MustUpdateGrounding();
        }

        public override void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
            CurrentMovementState.OnGroundHit(hitCollider, hitNormal, hitPoint, isStableOnHit);
        }

        public override void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
            CurrentMovementState.OnMovementHit(hitCollider, hitNormal, hitPoint, isStableOnHit);
        }

        /// <summary>
        /// This is called by MyPlayer upon jump input press
        /// </summary>
        public void OnJump()
        {
            if(CurrentMovementState == DefaultMovementState)
            {
                DefaultMovementState.OnJump();
            }
        }

        /// <summary>
        /// This is called by MyPlayer upon crouch input press
        /// </summary>
        public void OnCrouch(bool crouch)
        {
            if (CurrentMovementState == DefaultMovementState)
            {
                DefaultMovementState.OnCrouch(crouch);
            }
        }

        public void AddVelocity(Vector3 velocity)
        {
            if (CurrentMovementState == DefaultMovementState)
            {
                DefaultMovementState.AddVelocity(velocity);
            }
        }

        // This is called by MyPlayer upon charging input
        public void Charge()
        {
            if (CurrentMovementState == DefaultMovementState)
            {
                TransitionToState(ChargingState);
            }
        }
    }
}