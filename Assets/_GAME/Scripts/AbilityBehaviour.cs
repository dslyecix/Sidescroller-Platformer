using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class AbilityBehaviour : ScriptableObject
{
    // Interface for classes that represent an effect that
    // an ability will have.  Apply a force, change a stat
    // swap places with, etc.
    
    public abstract void Run(List<AbilityTarget> targets);
}
