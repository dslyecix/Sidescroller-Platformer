﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController
{
    public abstract class BaseMoverController : MonoBehaviour
    {
        // The PhysicsMover that will be assigned to this MoverController via the inspector
        public PhysicsMover PhysicsMover { get; private set; }

        /// <summary>
        /// This is called by the PhysicsMover in its Awake to setup references
        /// </summary>
        public void SetupMover(PhysicsMover mover)
        {
            PhysicsMover = mover;
        }

        /// <summary>
        /// Asks for the new position and rotation that the mover should have on this update
        /// </summary>
        public abstract void UpdateMovement(out Vector3 goalPosition, out Quaternion goalRotation, float deltaTime);
    }
}