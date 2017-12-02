using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace KinematicCharacterController.Walkthrough.RootMotionExample
{
    [System.Serializable]
    public class MyRootMotionMovementState : MyMovementState
    {
        [Header("Air Movement")]
        public float MaxAirMoveSpeed = 10f;
        public float AirAccelerationSpeed = 5f;
        public float Drag = 0.1f;

        [Header("Misc")]
        public Vector3 Gravity = new Vector3(0, -30f, 0);
        
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
            currentRotation = AssignedCharacterController.RootMotionRotationDelta * currentRotation;
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            if (KinematicCharacterMotor.IsStableOnGround)
            {
                if (deltaTime > 0)
                {
                    // The final velocity is the velocity from root motion reoriented on the ground plane
                    currentVelocity = AssignedCharacterController.RootMotionPositionDelta / deltaTime;
                    currentVelocity = KinematicCharacterMotor.GetDirectionTangentToSurface(currentVelocity, KinematicCharacterMotor.GroundNormal) * currentVelocity.magnitude;
                }
                else
                {
                    // Prevent division by zero
                    currentVelocity = Vector3.zero;
                }
            }
            else
            {
                if (AssignedCharacterController.ForwardAxis > 0f)
                {
                    // If we want to move, add an acceleration to the velocity
                    Vector3 targetMovementVelocity = KinematicCharacterMotor.CharacterForward * AssignedCharacterController.ForwardAxis * MaxAirMoveSpeed;
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