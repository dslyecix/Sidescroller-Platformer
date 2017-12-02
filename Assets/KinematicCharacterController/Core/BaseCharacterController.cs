using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController
{
    public abstract class BaseCharacterController : MonoBehaviour
    {
        // The KinematicCharacterMotor that will be assigned to this CharacterController via the inspector
        public KinematicCharacterMotor KinematicCharacterMotor { get; private set; }

        /// <summary>
        /// This is called by the KinematicCharacterMotor in its Awake to setup references
        /// </summary>
        public void SetupCharacterMotor(KinematicCharacterMotor motor)
        {
            KinematicCharacterMotor = motor;
            motor.CharacterController = this;
        }

        /// <summary>
        /// Asks if the character should probe for ground on this character update (return true or false). 
        /// Note that if ground probing finds valid ground, the character will automatically snap to the
        /// ground surface.
        /// </summary>
        public abstract bool MustUpdateGrounding();

        /// <summary>
        /// Asks what the character's rotation should be on this character update. 
        /// Modify the "currentRotation" to change the character's rotation.
        /// </summary>
        public abstract void UpdateRotation(ref Quaternion currentRotation, float deltaTime);

        /// <summary>
        /// Asks what the character's velocity should be on this character update. 
        /// Modify the "currentVelocity" to change the character's velocity.
        /// </summary>
        public abstract void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime);

        /// <summary>
        /// Gives you a callback for before the character update begins, if you 
        /// want to do anything to start off the update.
        /// </summary>
        public abstract void BeforeCharacterUpdate(float deltaTime);

        /// <summary>
        /// Gives you a callback for when the character update has reached its end, if you 
        /// want to do anything to finalize the update.
        /// </summary>
        public abstract void AfterCharacterUpdate(float deltaTime);

        /// <summary>
        /// Asks if a given collider should be considered for character collisions.
        /// Useful for ignoring specific colliders in specific situations.
        /// </summary>
        public abstract bool IsColliderValidForCollisions(Collider coll);

        /// <summary>
        /// Asks if the character can stand stable on a given collider.
        /// </summary>
        public abstract bool CanBeStableOnCollider(Collider coll);

        /// <summary>
        /// Gives you a callback for ground probing hits
        /// </summary>
        public abstract void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit);

        /// <summary>
        /// Gives you a callback for character movement hits
        /// </summary>
        public abstract void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit);
    }
}