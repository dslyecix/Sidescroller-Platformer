using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RestrainToPlane : MonoBehaviour {

	public float offsetDistance = 5f;
	public GameObject mouseCursor;

	// Use this for initialization
	void Start () {
		 
	}
	
	// Update is called once per frame
	void Update () {
		transform.position = transform.parent.transform.position + Vector3.right * Vector3.Dot(transform.parent.forward * offsetDistance, Vector3.right);
	
	}
}
