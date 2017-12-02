using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

public class PlayerInput : MonoBehaviour
{

    public PlayerCharacterController Character;
    public float MouseSensitivity = 0.02f;
    public float attackCooldown = 5;
    public Collider[] IgnoredColliders;

    private Vector3 _moveInputVector = Vector3.zero;
    private Vector3 _lookInputVector = Vector3.zero;

   
    private Cinemachine.CinemachineVirtualCamera camera;
    private bool canAttack;
    
    private float attackCooldownTimer;

    private void Start()
    {
        //Cursor.lockState = CursorLockMode.Locked;
        attackCooldownTimer = attackCooldown;
        canAttack = true;
        Character.IgnoredColliders = IgnoredColliders;

        camera = FindObjectOfType<Cinemachine.CinemachineVirtualCamera>();
    }

    private void Update()
    {
       
        if (Input.GetMouseButtonDown(0))
        {
            //Cursor.lockState = CursorLockMode.Locked;
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

        if (Character)
        {
            // Apply move input to character
            Vector3 worldSpaceInput = Quaternion.LookRotation(Vector3.forward,Vector3.up) * _moveInputVector;
            Vector3 lookDirection = new Vector3(worldSpaceInput.x, 0, 0);
            Character.Walk(isWalking);
            Character.SetInputs(worldSpaceInput, lookDirection);

            // Jump input
            if (Input.GetKeyDown(KeyCode.Space))
            {
                Character.Jump();
            }

            if (!canAttack) {
                attackCooldownTimer -= Time.deltaTime;
                if (attackCooldownTimer <=0f) canAttack = true;
            } 

            //Attack input?
            if (Input.GetKeyDown(KeyCode.F))
            {
                if (canAttack) {
                    Character.Attack();
                    attackCooldownTimer = attackCooldown;
                    canAttack = false;
                } 
            }




            // Croucing input
            if (Input.GetKeyDown(KeyCode.C))
            {
                Character.Crouch(true);
            }
            else if (Input.GetKeyUp(KeyCode.C))
            {
                Character.Crouch(false);
            }

            // Apply input to camera
            float scrollInput = -Input.GetAxis("Mouse ScrollWheel");
            #if UNITY_WEBGL
            scrollInput = 0f;
            #endif
        }
    }
}
