using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.SwimmingState
{
    [System.Serializable]
    public class MyChargingState : MyMovementState
    {
        public float ChargeSpeed = 15f;
        public float MaxChargeTime = 1.5f;
        public float StoppedTime = 1f;
        public Vector3 Gravity = new Vector3(0, -30f, 0);

        private Vector3 _currentChargeVelocity;
        private bool _isStopped;
        private bool _mustStopVelocity = false;
        private float _timeSinceStartedCharge = 0;
        private float _timeSinceStopped = 0;

        public override void AfterCharacterUpdate(float deltaTime)
        {
            // Detect being stopped by elapsed time
            if (!_isStopped && _timeSinceStartedCharge > MaxChargeTime)
            {
                _mustStopVelocity = true;
                _isStopped = true;
            }

            // Detect end of stopping phase and transition back to default movement state
            if (_timeSinceStopped > StoppedTime)
            {
                AssignedCharacterController.TransitionToState(AssignedCharacterController.DefaultMovementState);
            }
        }

        public override void BeforeCharacterUpdate(float deltaTime)
        {
            // Update times
            _timeSinceStartedCharge += deltaTime;
            if (_isStopped)
            {
                _timeSinceStopped += deltaTime;
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
            // Detect being stopped by obstructions
            if(!_isStopped && !isStableOnHit)
            {
                _mustStopVelocity = true;
                _isStopped = true;
            }
        }

        public override void OnStateEnter(MyMovementState previousState)
        {
            _currentChargeVelocity = KinematicCharacterMotor.CharacterTransform.forward * ChargeSpeed;
            _isStopped = false;
            _timeSinceStartedCharge = 0f;
            _timeSinceStopped = 0f;
        }

        public override void OnStateExit(MyMovementState nextState)
        {
        }

        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            // If we have stopped and need to cancel velocity, do it here
            if(_mustStopVelocity)
            {
                currentVelocity = Vector3.zero;
                _mustStopVelocity = false;
            }

            if(_isStopped)
            {
                // When stopped, do no velocity handling except gravity
                currentVelocity += Gravity * deltaTime;
            }
            else
            {
                // When charging, velocity is always constant
                currentVelocity = _currentChargeVelocity;
            }
        }
    }
}