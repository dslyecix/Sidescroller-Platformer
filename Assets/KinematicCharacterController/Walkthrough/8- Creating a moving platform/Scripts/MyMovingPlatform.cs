using KinematicCharacterController;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Playables;

namespace KinematicCharacterController.Walkthrough.MovingPlatform
{
    public class MyMovingPlatform : BaseMoverController
    {
        public Animator Animator;

        private Transform _transform;
        private PlayableGraph _playableGraph;

        private void Start()
        {
            _transform = this.transform;

            // Get the PlayableGraph of the Animator and stop is, so we can maunally control its evaluation later
            _playableGraph = Animator.playableGraph;
            _playableGraph.Stop();
        }

        public override void UpdateMovement(out Vector3 goalPosition, out Quaternion goalRotation, float deltaTime)
        {
            // Remember pose before animation
            Vector3 _positionBeforeAnim = _transform.position;
            Quaternion _rotationBeforeAnim = _transform.rotation;

            // Update animation
            _playableGraph.Evaluate(deltaTime);

            // Set our platform's goal pose to the animation's
            goalPosition = _transform.position;
            goalRotation = _transform.rotation;

            // Reset the actual transform pose to where it was before evaluating. 
            // This is so that the real movement can be handled by the physics mover; not the animation
            _transform.position = _positionBeforeAnim;
            _transform.rotation = _rotationBeforeAnim;
        }
    }
}