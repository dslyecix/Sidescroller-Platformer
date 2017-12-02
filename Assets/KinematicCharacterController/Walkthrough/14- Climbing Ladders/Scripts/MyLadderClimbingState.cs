using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.ClimbingLadders
{
    [System.Serializable]
    public class MyLadderClimbingState : MyMovementState
    {
        private enum ClimbingState
        {
            Anchoring,
            Climbing,
            DeAnchoring
        }

        public float AnchoringDuration = 1f;
        public float ClimbingSpeed = 3f;

        public MyLadder ActiveLadder { get; set; }

        private ClimbingState _internalClimbingState;
        private ClimbingState _climbingState
        {
            get
            {
                return _internalClimbingState;
            }
            set
            {
                _internalClimbingState = value;
                _anchoringTimer = 0f;
                _anchoringStartPosition = KinematicCharacterMotor.TransientPosition;
                _anchoringStartRotation = KinematicCharacterMotor.TransientRotation;
            }
        }

        private Vector3 _targetPosition;
        private Quaternion _targetRotation;
        private float _onLadderSegmentState = 0;
        private float _anchoringTimer = 0f;
        private Vector3 _anchoringStartPosition = Vector3.zero;
        private Quaternion _anchoringStartRotation = Quaternion.identity;

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

        public override void OnStateEnter(MyMovementState previousState)
        {
            KinematicCharacterMotor.HandlePhysics(false, true);
            _climbingState = ClimbingState.Anchoring;

            // Store the target position and rotation to snap to
            _targetPosition = ActiveLadder.ClosestPointOnLadderSegment(KinematicCharacterMotor.TransientPosition, out _onLadderSegmentState);
            _targetRotation = ActiveLadder.transform.rotation;
        }

        public override void OnStateExit(MyMovementState nextState)
        {
            KinematicCharacterMotor.HandlePhysics(true, true);
        }

        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            switch (_climbingState)
            {
                case ClimbingState.Climbing:
                    currentRotation = ActiveLadder.transform.rotation;
                    break;
                case ClimbingState.Anchoring:
                case ClimbingState.DeAnchoring:
                    currentRotation = Quaternion.Slerp(_anchoringStartRotation, _targetRotation, (_anchoringTimer / AnchoringDuration));
                    break;
            }
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            currentVelocity = Vector3.zero;
            
            switch (_climbingState)
            {
                case ClimbingState.Climbing:
                    currentVelocity = (AssignedCharacterController.LadderUpDownInput * ActiveLadder.transform.up).normalized * ClimbingSpeed;
                    break;
                case ClimbingState.Anchoring:
                case ClimbingState.DeAnchoring:
                    Vector3 tmpPosition = Vector3.Lerp(_anchoringStartPosition, _targetPosition, (_anchoringTimer / AnchoringDuration));
                    currentVelocity = KinematicCharacterMotor.GetVelocityForMovePosition(tmpPosition, deltaTime);
                    break;
            }
        }

        public override void AfterCharacterUpdate(float deltaTime)
        {
            switch (_climbingState)
            {
                case ClimbingState.Climbing:
                    // Detect getting off ladder during climbing
                    ActiveLadder.ClosestPointOnLadderSegment(KinematicCharacterMotor.TransientPosition, out _onLadderSegmentState);
                    if (Mathf.Abs(_onLadderSegmentState) > 0.05f)
                    {
                        _climbingState = ClimbingState.DeAnchoring;

                        // If we're higher than the ladder top point
                        if (_onLadderSegmentState > 0)
                        {
                            _targetPosition = ActiveLadder.TopReleasePoint.position;
                            _targetRotation = ActiveLadder.TopReleasePoint.rotation;
                        }
                        // If we're lower than the ladder bottom point
                        else if (_onLadderSegmentState < 0)
                        {
                            _targetPosition = ActiveLadder.BottomReleasePoint.position;
                            _targetRotation = ActiveLadder.BottomReleasePoint.rotation;
                        }
                    }
                    break;
                case ClimbingState.Anchoring:
                case ClimbingState.DeAnchoring:
                    // Detect transitioning out from anchoring states
                    if (_anchoringTimer >= AnchoringDuration)
                    {
                        if (_climbingState == ClimbingState.Anchoring)
                        {
                            _climbingState = ClimbingState.Climbing;
                        }
                        else if (_climbingState == ClimbingState.DeAnchoring)
                        {
                            AssignedCharacterController.TransitionToState(AssignedCharacterController.DefaultMovementState);
                        }
                    }

                    // Keep track of time since we started anchoring
                    _anchoringTimer += deltaTime;
                    break;
            }
        }

        public override void BeforeCharacterUpdate(float deltaTime)
        {
        }
    }
}