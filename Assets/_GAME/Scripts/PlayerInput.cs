using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

public class PlayerInput : MonoBehaviour
{

    public PlayerCharacterController character;
    public float MouseSensitivity = 0.02f;
    public float attackCooldown = 5;
    public Collider[] IgnoredColliders;

    public ParticleSystem jumpParticles;

    private Vector3 _moveInputVector = Vector3.zero;
    private Vector3 _lookInputVector = Vector3.zero;

   
    private Cinemachine.CinemachineVirtualCamera camera;
    private bool canAttack;
    
    private float attackCooldownTimer;

    private void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;
        attackCooldownTimer = attackCooldown;
        canAttack = true;
        character.IgnoredColliders = IgnoredColliders;

        camera = FindObjectOfType<Cinemachine.CinemachineVirtualCamera>();
    }

    private void Update()
    {
       
        if (Input.GetMouseButtonDown(0))
        {
            Cursor.lockState = CursorLockMode.Locked;
        }

        // Gather input
        float moveAxisForward = Input.GetAxisRaw("Vertical");
        float moveAxisRight = Input.GetAxisRaw("Horizontal");

        _moveInputVector = new Vector3(moveAxisRight, 0f, moveAxisForward);
        _moveInputVector = Vector3.ClampMagnitude(_moveInputVector, 1f);

        bool isWalking = Input.GetKey(KeyCode.LeftShift);


        if (Cursor.lockState != CursorLockMode.Locked)
        {
            //_lookInputVector = Vector3.zero;
        }

        if (character)
        {
            // Apply move input to character
            // Vector3 worldSpaceInput = Quaternion.LookRotation(Vector3.forward,Vector3.up) * _moveInputVector;
            // Vector3 lookDirection = new Vector3(worldSpaceInput.x, 0, 0);
            // character.Walk(isWalking);
            // character.SetInputs(worldSpaceInput, lookDirection);

            Vector3 localCameraRelativeInput = Quaternion.LookRotation(camera.transform.forward,Vector3.up) * _moveInputVector;
            Vector3 lookDirection = localCameraRelativeInput;
            character.Walk(isWalking);
            character.SetInputs(localCameraRelativeInput, lookDirection);

            // Jump input
            if (Input.GetKeyDown(KeyCode.Space))
            {
                character.Jump();
            }

            if (!canAttack) {
                attackCooldownTimer -= Time.deltaTime;
                if (attackCooldownTimer <=0f) canAttack = true;
            } 

            if (Input.GetKeyDown(KeyCode.Q))
            {
                character.DoAFlip();
            }


            //Attack input?
            if (Input.GetKeyDown(KeyCode.F))
            {
                if (canAttack) {
                    character.Attack();
                    attackCooldownTimer = attackCooldown;
                    canAttack = false;
                } 
            }




            // Croucing input
            if (Input.GetKeyDown(KeyCode.C))
            {
                character.Crouch(true);
            }
            else if (Input.GetKeyUp(KeyCode.C))
            {
                character.Crouch(false);
            }

            // Apply input to camera
            float scrollInput = -Input.GetAxis("Mouse ScrollWheel");
            #if UNITY_WEBGL
            scrollInput = 0f;
            #endif
        }
    }

    public void PlayParticles(ParticleSystem _particleSystem) {
        _particleSystem.Play(); 
    }
}
