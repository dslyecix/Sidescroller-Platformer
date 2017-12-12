using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IAbilityBehaviour
{
    // Interface for classes that represent an effect that
    // an ability will have.  Apply a force, change a stat
    // swap places with, etc.
    



    void Run(List<IAbilityTarget> targets, Transform _caster);
}
