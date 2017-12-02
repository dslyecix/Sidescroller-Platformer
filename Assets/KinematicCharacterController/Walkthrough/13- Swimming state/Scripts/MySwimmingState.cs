using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.SwimmingState
{
    [System.Serializable]
    public class MySwimmingState : MyMovementState
    {
        public float MaxMoveSpeed = 5f;
        public float MovementSharpness = 2f;

        public Collider WaterZone { get; set; }

        public override void AfterCharacterUpdate(float deltaTime)
        {
        }

        public override void BeforeCharacterUpdate(float deltaTime)
        {
        }

        public override bool CanBeStableOnCollider(Collider coll)
        {
            return false;
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
        }

        public override void OnStateExit(MyMovementState nextState)
        {
        }

        public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
        }

        public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            // Smoothly interpolate to target swimming velocity
            Vector3 targetMovementVelocity = (AssignedCharacterController.WorldspaceMoveInputVector + (KinematicCharacterMotor.CharacterUp * AssignedCharacterController.VerticalInput)).normalized * MaxMoveSpeed;
            Vector3 smoothedVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-MovementSharpness * deltaTime));

            // See if our swimming reference point would be out of water after the movement from our velocity has been applied
            {
                Vector3 resultingSwimmingReferancePosition = KinematicCharacterMotor.TransientPosition + (smoothedVelocity * deltaTime) + (AssignedCharacterController.SwimmingReferencePoint.position - KinematicCharacterMotor.TransientPosition);
                Vector3 closestPointWaterSurface = Physics.ClosestPoint(resultingSwimmingReferancePosition, WaterZone, WaterZone.transform.position, WaterZone.transform.rotation);

                // if our position would be outside the water surface on next update, project the velocity on the surface normal so that it would not take us out of the water
                if (closestPointWaterSurface != resultingSwimmingReferancePosition)
                {
                    Vector3 waterSurfaceNormal = (resultingSwimmingReferancePosition - closestPointWaterSurface).normalized;
                    smoothedVelocity = Vector3.ProjectOnPlane(smoothedVelocity, waterSurfaceNormal);
                }
            }

            currentVelocity = smoothedVelocity;
        }
    }
}