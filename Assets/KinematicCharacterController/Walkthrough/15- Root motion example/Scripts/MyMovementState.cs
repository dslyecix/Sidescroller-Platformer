using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Walkthrough.RootMotionExample
{
    public abstract class MyMovementState
    {
        [HideInInspector]
        public MyCharacterController AssignedCharacterController;
        [HideInInspector]
        public KinematicCharacterMotor KinematicCharacterMotor;

        public abstract void OnStateExit(MyMovementState nextState);
        public abstract void OnStateEnter(MyMovementState previousState);

        public abstract bool MustUpdateGrounding();
        public abstract void UpdateRotation(ref Quaternion currentRotation, float deltaTime);
        public abstract void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime);
        public abstract void BeforeCharacterUpdate(float deltaTime);
        public abstract void AfterCharacterUpdate(float deltaTime);
        public abstract bool IsColliderValidForCollisions(Collider coll);
        public abstract bool CanBeStableOnCollider(Collider coll);
        public abstract void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit);
        public abstract void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit);
    }
}