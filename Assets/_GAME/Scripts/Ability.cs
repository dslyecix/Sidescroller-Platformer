using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
public class Ability : ScriptableObject {

	public Transform casterOfOrigin;

	public IAbilityTarget target;
	public List<IAbilityBehaviour> behaviours = new List<IAbilityBehaviour>();
	List<IAbilityTarget> behaviourTargets;

	void CastAbility(Transform caster)
	{
		casterOfOrigin = caster;

		ResolveTargets();

		behaviourTargets.Add(target);
		

		ExecuteBehaviours();
	}

	void ResolveTargets() 
	{
		if (target != null) {
			target.FetchTargets();
		}
	}

	void ExecuteBehaviours() 
	{
		foreach (var behaviour in behaviours)
		{
			behaviour.Run(behaviourTargets, casterOfOrigin);
		}
	}
}
