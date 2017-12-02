using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController.Examples
{
    public class ExampleAIController : MonoBehaviour
    {
        public float MovementPeriod = 1f;
        public ExampleCharacterController[] Characters;
        
        private void Update()
        {
            // Simulate an input on all controlled characters
            Vector3 moveInputVector = Mathf.Sin(Time.time * MovementPeriod) * Vector3.forward;
            Vector3 lookInputVector = Vector3.Slerp(-Vector3.forward, Vector3.forward, moveInputVector.z).normalized;
            for (int i = 0; i < Characters.Length; i++)
            {
                Characters[i].SetInputs(moveInputVector, lookInputVector);
            }
        }
    }
}