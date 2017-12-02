using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController
{
    [RequireComponent(typeof(Rigidbody))]
    public class PhysicsMover : MonoBehaviour
    {
        [ReadOnly]
        public Rigidbody MoverRigidbody;
        public BaseMoverController MoverController;
        
        private Transform _transform;
        private Vector3 _goalPosition;
        private Quaternion _goalRotation;
        private Vector3 _lastUpdatePosition;
        private Quaternion _lastUpdateRotation;

        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        public void ValidateData()
        {
            MoverRigidbody = gameObject.GetComponent<Rigidbody>();

            MoverRigidbody.centerOfMass = Vector3.zero;
            MoverRigidbody.useGravity = false;
            MoverRigidbody.drag = 0f;
            MoverRigidbody.angularDrag = 0f;
            MoverRigidbody.maxAngularVelocity = Mathf.Infinity;
            MoverRigidbody.maxDepenetrationVelocity = Mathf.Infinity;
            MoverRigidbody.collisionDetectionMode = CollisionDetectionMode.Discrete;
            MoverRigidbody.isKinematic = true;
            MoverRigidbody.constraints = RigidbodyConstraints.None;
            MoverRigidbody.interpolation = KinematicCharacterSystem.UseInterpolation ? RigidbodyInterpolation.Interpolate : RigidbodyInterpolation.None;
        }

        private void OnEnable()
        {
            KinematicCharacterSystem.GetOrCreateInstance().PhysicsMovers.Add(this);
        }

        private void OnDisable()
        {
            if (KinematicCharacterSystem.HasInstance())
            {
                KinematicCharacterSystem.GetInstance().PhysicsMovers.Remove(this);
            }
        }

        private void Awake()
        {
            _transform = this.transform;
            MoverController.SetupMover(this);

            _goalPosition = MoverRigidbody.position;
            _goalRotation = MoverRigidbody.rotation;
            _lastUpdatePosition = MoverRigidbody.position;
            _lastUpdateRotation = MoverRigidbody.rotation;

            MoverRigidbody.interpolation = KinematicCharacterSystem.UseInterpolation ? RigidbodyInterpolation.Interpolate : RigidbodyInterpolation.None;
        }

        /// <summary>
        /// Caches velocity values based on deltatime and target position/rotations
        /// </summary>
        public void CalculateVelocities(float deltaTime)
        {
            _lastUpdatePosition = _goalPosition;
            _lastUpdateRotation = _goalRotation;

            MoverController.UpdateMovement(out _goalPosition, out _goalRotation, deltaTime);

            if (deltaTime > 0f)
            {
                MoverRigidbody.velocity = (_goalPosition - _lastUpdatePosition) / deltaTime;
                
                Quaternion rotationFromCurrentToGoal = _goalRotation * (Quaternion.Inverse(_lastUpdateRotation));
                MoverRigidbody.angularVelocity = (Mathf.Deg2Rad * rotationFromCurrentToGoal.eulerAngles) / deltaTime;
            }
        }

        /// <summary>
        /// Simulates placing the mover at its goal immediately
        /// </summary>
        public void SimulateAtGoal()
        {
#if UNITY_2017_2_OR_NEWER
            if (!Physics.autoSyncTransforms)
            {
                MoverRigidbody.position = _goalPosition;
                MoverRigidbody.rotation = _goalRotation;
            }
#endif
            _transform.position = _goalPosition;
            _transform.rotation = _goalRotation;
        }

        /// <summary>
        /// Restores the mover's position/rotation to their last recorded values
        /// </summary>
        public void Desimulate()
        {
#if UNITY_2017_2_OR_NEWER
            if (!Physics.autoSyncTransforms)
            {
                MoverRigidbody.position = _lastUpdatePosition;
                MoverRigidbody.rotation = _lastUpdateRotation;
            }
#endif
            _transform.position = _lastUpdatePosition;
            _transform.rotation = _lastUpdateRotation;
        }

        /// <summary>
        /// Moves the mover with interpolation and recorde position/rotation values
        /// </summary>
        public void UpdateMovement()
        {
            MoverRigidbody.MovePosition(_goalPosition);
            MoverRigidbody.MoveRotation(_goalRotation);
        }
    }
}