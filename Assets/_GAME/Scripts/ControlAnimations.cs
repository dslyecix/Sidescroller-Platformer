using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlAnimations : MonoBehaviour {

	Animator animator;
	Rigidbody rigidbody;
	PlayerCharacterController cc;

	// Use this for initialization
	void Start () {
		animator = GetComponentInChildren<Animator>();
		rigidbody = GetComponent<Rigidbody>();
		cc = GetComponent<PlayerCharacterController>();
	}
	
	// Update is called once per frame
	void Update () {
		animator.SetFloat("speedPercent",rigidbody.velocity.magnitude / cc.MaxStableMoveSpeed);
	}
}
