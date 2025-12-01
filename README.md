# Godot4-CharacterBody2DtoGDScript
 A conversion of the CharacterBody2D C++ code to GDScript for whatever you want.

 The original in the Godot source code in [physics_body_2d.cpp](https://github.com/godotengine/godot/blob/4.5/scene/2d/physics/character_body_2d.cpp).

## Why would I want to use this?
 You may or may not like how CharacterBody2D works, this can allow you to make modifications to how `move_and_slide()` works and you can tweak it however you'd like. :3

## Issues
* It doesn't inherit PhysicsBody2D, as I can not get that to work for whatever reason, so it inherits AnimatableBody2D. 
* It's not fully 1to1 as a function needed to be changed due to a possible oversight(?) relating to PhysicsTestMotionResult2D and KinematicCollision2D.
