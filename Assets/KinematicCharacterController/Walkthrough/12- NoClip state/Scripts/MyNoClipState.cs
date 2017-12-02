using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.NoClipState
{
    [System.Serializable]
    public class MyNoClipState : MyMovementState
    {
        public float MaxMoveSpeed = 10f;
        public float MovementSharpness = 15;

        public override void AfterCharacterUpdate(float deltaTime)
        {
        }

        public override void BeforeCharacterUpdate(float deltaTime)
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
            return false;
        }

        public override void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
        }

        public override void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
        {
        }

        public override void OnStateEnter(MyMovementState previousState)
        {
            KinematicCharacterMotor.HandlePhysics(false, false);
        }

        public override void OnStateExit(MyMovementState nextState)
        {
            KinematicCharacterMotor.HandlePhysics(true, true);
        }

        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            // Smoothly interpolate to target velocity
            Vector3 targetMovementVelocity = (AssignedCharacterController.WorldspaceMoveInputVector + (KinematicCharacterMotor.CharacterUp * AssignedCharacterController.VerticalInput)).normalized * MaxMoveSpeed;
            currentVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-MovementSharpness * deltaTime));
        }
    }
}