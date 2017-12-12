using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
public class Fireball : Ability {

	//public Transform caster;

	public List<AbilityTarget> targets;
	public List<AbilityBehaviour> behaviours;

	List<AbilityTarget> behaviourTargets;

	public override void CastAbility()
	{
		Debug.Log("Cast Ability fired");
	 	//caster = _caster;

		ResolveTargets();

		behaviourTargets = targets;
		
		ExecuteBehaviours();
	}

	public override void ResolveTargets() 
	{
		if (targets != null) {
			foreach (var target in targets)
			{
				Debug.Log("Resolving targets with " + target.name);
				target.FetchTargets();
			}
				
		} else {
			Debug.Log("No target!");
		}
	}

	public override void ExecuteBehaviours() 
	{
		foreach (var behaviour in behaviours)
		{
			Debug.Log("Resolving behaviours with " + behaviour.name);
			behaviour.Run(behaviourTargets);
		}
	}

}
