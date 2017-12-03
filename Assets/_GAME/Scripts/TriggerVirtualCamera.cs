using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TriggerVirtualCamera : MonoBehaviour {

	public Cinemachine.CinemachineVirtualCamera cam;
	public int cameraTriggerPriority;

	private int cameraInitPriority = 0;

	void Start () {
		cam.Priority = cameraInitPriority;
	}


	void OnTriggerStay() {
		cam.Priority = cameraTriggerPriority;
	}

	void OnTriggerExit() {
		cam.Priority = cameraInitPriority;
	}

}
