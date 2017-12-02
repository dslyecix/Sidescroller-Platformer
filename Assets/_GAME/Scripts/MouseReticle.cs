using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MouseReticle : MonoBehaviour {

	public Camera camera;
    
	Vector3 mousePos;
	Plane reticlePlane;

	// Use this for initialization
	void Start () {
		reticlePlane = new Plane(Vector3.forward, Vector3.zero);
	}
	
	// Update is called once per frame
	void Update () {
		Ray ray = camera.ScreenPointToRay(Input.mousePosition);
		float hitDistance;
		if (reticlePlane.Raycast(ray, out hitDistance)) 
		{
			transform.position = camera.transform.position + ray.direction * hitDistance;
		}

		
	}
}
