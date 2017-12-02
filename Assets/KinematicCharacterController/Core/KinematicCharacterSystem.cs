using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace KinematicCharacterController
{
    public class KinematicCharacterSystem : MonoBehaviour
    {
        [HideInInspector]
        [NonSerialized]
        public List<KinematicCharacterMotor> KinematicCharacterMotors = new List<KinematicCharacterMotor>();
        [HideInInspector]
        [NonSerialized]
        public List<PhysicsMover> PhysicsMovers = new List<PhysicsMover>();

        public const bool UseInterpolation = true;

        #region Singleton Section
        private static KinematicCharacterSystem _instance;

        public static KinematicCharacterSystem GetOrCreateInstance()
        {
            if (_instance == null)
            {
                GameObject systemGameObject = new GameObject("KinematicCharacterSystem");
                _instance = systemGameObject.AddComponent<KinematicCharacterSystem>();

                systemGameObject.hideFlags = HideFlags.NotEditable;
                _instance.hideFlags = HideFlags.NotEditable;
            }

            return _instance;
        }

        public static KinematicCharacterSystem GetInstance()
        {
            return _instance;
        } 

        public static bool HasInstance()
        {
            return _instance != null; 
        }

        // This is to prevent duplicating the singleton gameobject on script recompiles
        private void OnDisable()
        {
            Destroy(this.gameObject);
        }
        #endregion
        
        private void FixedUpdate()
        {
#pragma warning disable 0162
            float deltaTime = Time.deltaTime;
            int moversCount = PhysicsMovers.Count;
            int motorsCount = KinematicCharacterMotors.Count;

            // Update PhysicsMover velocities
            for (int i = 0; i < moversCount; i++)
            {
                PhysicsMovers[i].CalculateVelocities(deltaTime);
            }

            // Character controller update phase 1
            for (int i = 0; i < motorsCount; i++)
            {
                KinematicCharacterMotors[i].CharacterUpdatePhase1(deltaTime);
            }

            // Simulate PhysicsMover displacement
            for (int i = 0; i < moversCount; i++)
            {
                PhysicsMovers[i].SimulateAtGoal();
            }

            // Character controller update phase 2
            for (int i = 0; i < motorsCount; i++)
            {
                KinematicCharacterMotors[i].CharacterUpdatePhase2(deltaTime);
            }

            if (UseInterpolation)
            {
                // Character controller update phase 3 (it is important to call those once ALL CharacterUpdatePhase2s are done)
                for (int i = 0; i < motorsCount; i++)
                {
                    KinematicCharacterMotors[i].CharacterUpdatePhase3(deltaTime);
                }

                // Desimulate PhysicsMover displacement
                for (int i = 0; i < moversCount; i++)
                {
                    PhysicsMovers[i].Desimulate();
                }

#if UNITY_2017_2_OR_NEWER
                // Sync rigidbody transforms manually
                Physics.SyncTransforms();
#endif

                // Update true interpolated movement
                for (int i = 0; i < moversCount; i++)
                {
                    PhysicsMovers[i].UpdateMovement();
                }
            }
#pragma warning restore 0162
        }
    }
}