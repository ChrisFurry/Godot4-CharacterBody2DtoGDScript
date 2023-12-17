# Godot4-CharacterBody2DtoGDScript
 A conversion of the CharacterBody2D C++ code to GDScript for whatever you want.

 Because yes I am that insane. This took me 6 hours.

 The original in the Godot source code in [physics_body_2d.cpp](https://github.com/godotengine/godot/blob/4.2/scene/2d/physics_body_2d.cpp) and starts at line 1104 and ends at 1830.

## Why would I want to use this?
 You may or may not like how CharacterBody2D works, this can allow you to make modifications to how `move_and_slide()` works and you can tweak it however you'd like. :3

## Issues
* It doesn't inherit PhysicsBody2D, as I can not get that to work for whatever reason, so it inherits CharacterBody2D. 
* It's not fully 1to1 as some functions or variables needed to be changed.