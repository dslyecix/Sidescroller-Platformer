using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.RootMotionExample
{
    public class MyCharacterController : BaseCharacterController
    {
        public Animator CharacterAnimator;
        public MyRootMotionMovementState RootMotionMovementState = new MyRootMotionMovementState();

        [Header("Animation Parameters")]
        public float ForwardAxisSharpness = 10;
        public float TurnAxisSharpness = 5;

        public float ForwardAxis { get; private set; }
        public float RightAxis { get; private set; }
        public MyMovementState CurrentMovementState { get; private set; }

        // RootMotion deltas
        public Vector3 RootMotionPositionDelta { get; private set; }
        public Quaternion RootMotionRotationDelta { get; private set; }

        private float _targetForwardAxis;
        private float _targetRightAxis;

        private void Start()
        {
            // Handle initial state
            TransitionToState(RootMotionMovementState);

            RootMotionPositionDelta = Vector3.zero;
            RootMotionRotationDelta = Quaternion.identity;
        }

        private void Update()
        {
            // Handle animation
            ForwardAxis = Mathf.Lerp(ForwardAxis, _targetForwardAxis, 1f - Mathf.Exp(-ForwardAxisSharpness * Time.deltaTime));
            RightAxis = Mathf.Lerp(RightAxis, _targetRightAxis, 1f - Mathf.Exp(-TurnAxisSharpness * Time.deltaTime));
            CharacterAnimator.SetFloat("Forward", ForwardAxis);
            CharacterAnimator.SetFloat("Turn", RightAxis);
            CharacterAnimator.SetBool("OnGround", KinematicCharacterMotor.IsStableOnGround);
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
        /// This is called every frame by MyPlayer in order to tell the character where to go and where to look
        /// </summary>
        public void SetAxisInputs(float forwardAxis, float rightAxis)
        {
            _targetForwardAxis = forwardAxis;
            _targetRightAxis = rightAxis;
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called before the character begins its movement update
        /// </summary>
        public override void BeforeCharacterUpdate(float deltaTime)
        {
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

            // Reset root motion deltas
            RootMotionPositionDelta = Vector3.zero;
            RootMotionRotationDelta = Quaternion.identity;
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

        private void OnAnimatorMove()
        {
            // Accumulate rootMotion deltas between character updates 
            RootMotionPositionDelta += CharacterAnimator.deltaPosition;
            RootMotionRotationDelta = CharacterAnimator.deltaRotation * RootMotionRotationDelta;
        }
    }
}