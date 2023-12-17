# For now it inherits CharacterBody2D, since inheriting PhysicsBody2D causes it to not be usable.
extends CharacterBody2D;
# If you can find a way to inherit PhysicsBody2D then uncomment the variable/constant definitions below.

#var floor_block_on_wall:bool = true;
#var floor_constant_speed:bool = false;
#var floor_max_angle:float = 0.785398;
#var floor_snap_length:float = 1.0;
#var floor_stop_on_slope:bool = true;
#var max_slides:int = 4;
#var motion_mode:MotionMode = 0;
#var platform_floor_layers:int = 4294967295;
#var platform_on_leave:PlatformOnLeave = 0;
#var platform_wall_layers:int = 0;
#var safe_margin:float = 0.08;
#var slide_on_ceiling:bool = true;
#var up_direction:Vector2 = Vector2(0,-1);
#var velocity:Vector2 = Vector2(0,0);
#var wall_min_slide_angle:float = 0.261799;

# Internal Collision Variables
var previous_position:Vector2 = Vector2();

var real_velocity:Vector2 = Vector2();

var platform_velocity:Vector2 = Vector2();
var platform_rid:RID = RID();
var platform_layer:int;

var platform_object_id:int;

var motion_results:Array;
var last_motion:Vector2 = Vector2();

var on_floor:bool = false;
var floor_normal:Vector2 = Vector2();

var on_wall:bool = false;
var wall_normal:Vector2 = Vector2();

var on_ceiling:bool = false;

# So, if you pass 45 as limit, avoid numerical precision errors when angle is 45.
const FLOOR_ANGLE_THRESHOLD = 0.01;
# Taken from the math macro's file
const CMP_EPSILON = 0.00001;

func move_and_slide()->bool:
	var delta:float = get_physics_process_delta_time() if(Engine.is_in_physics_frame())else get_process_delta_time();
	
	var current_platform_velocity:Vector2 = get_platform_velocity();
	var gt:Transform2D = get_global_transform();
	previous_position = gt.get_origin();
	
	if((on_floor || on_wall) && platform_rid.is_valid()):
		var excluded:bool = false;
		if(on_floor):
			excluded = (platform_floor_layers & platform_layer) == 0;
		if(on_wall):
			excluded = (platform_wall_layers & platform_layer) == 0;
		if(!excluded):
			var bs:PhysicsDirectBodyState2D = PhysicsServer2D.body_get_direct_state(platform_rid);
			if(bs):
				var local_position:Vector2 = previous_position - bs.get_transform().get_origin();
				current_platform_velocity = bs.get_velocity_at_local_position(local_position);
			else:
				current_platform_velocity = Vector2();
				platform_rid = RID();
		else:
			current_platform_velocity = Vector2();
	
	motion_results.clear();
	last_motion = Vector2();
	
	var was_on_floor:bool = on_floor;
	on_floor = false;
	on_wall = false;
	on_ceiling = false;
	
	if(!current_platform_velocity.is_zero_approx()):
		var floor_result:KinematicCollision2D = move_and_collide(current_platform_velocity * delta,false,safe_margin,true);
		if(floor_result is KinematicCollision2D):
			motion_results.push_back(floor_result);
			_set_collision_direction(floor_result);
	# Motion Mode
	if(motion_mode == MOTION_MODE_GROUNDED):
		_move_and_slide_grounded(delta,was_on_floor);
	else:
		_move_and_slide_floating(delta);
	# Compute real velocity
	real_velocity = get_position_delta() / delta;
	
	if(platform_on_leave != PLATFORM_ON_LEAVE_DO_NOTHING):
		# Add last platform velocity when just left a moving platform.
		if(!on_floor && !on_wall):
			if(platform_on_leave == PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY && current_platform_velocity.dot(up_direction) < 0):
				current_platform_velocity = current_platform_velocity.slide(up_direction);
			velocity += current_platform_velocity;
	
	return motion_results.size() > 0;

func _move_and_slide_grounded(delta:float,was_on_floor:bool)->void:
	var motion:Vector2 = velocity * delta;
	var motion_slide_up = motion.slide(up_direction);
	
	var prev_floor_normal = floor_normal;
	
	platform_rid = RID();
	platform_object_id = -1;
	floor_normal = Vector2();
	platform_velocity = Vector2();
	
	# No sliding on first attempt to keep floor motion stable when possible,
	# When stop on slope is enabled or when there is no up direction.
	var sliding_enabled:bool = !floor_stop_on_slope;
	# Constant speed can be applied only the first time sliding is enabled.
	var can_apply_constant_speed:bool = sliding_enabled;
	#  If the platform's ceiling push down the body.
	var apply_ceiling_velocity:bool = false;
	var first_slide:bool = true;
	var vel_dir_facing_up:bool = velocity.dot(up_direction) > 0;
	var last_travel:Vector2;
	
	for i in max_slides:
		var prev_position:Vector2 = get_global_transform().origin;
		
		var result:KinematicCollision2D = move_and_collide(motion,false,safe_margin,true);
		var collided:bool = (result is KinematicCollision2D);
		
		last_motion = result.get_travel() if(collided)else motion;
		
		if(collided):
			motion_results.push_back(result);
			_set_collision_direction(result);
			
			# If we hit a ceiling platform, we set the vertical velocity to at least the platform one.
			if(on_ceiling && result.get_collider_velocity() != Vector2() && result.get_collider_velocity().dot(up_direction) < 0):
				# If ceiling sliding is on, only apply when the ceiling is flat or when the motion is upward.
				if(!slide_on_ceiling || motion.dot(up_direction) < 0 || (result.get_normal() + up_direction).length() < 0.01):
					apply_ceiling_velocity = true;
					var ceiling_vertical_velocity:Vector2 = up_direction * up_direction.dot(result.get_collider_velocity());
					var motion_vertical_velocity:Vector2 = up_direction * up_direction.dot(velocity);
					if(motion_vertical_velocity.dot(up_direction) > 0 || ceiling_vertical_velocity.length_squared() > motion_vertical_velocity.length_squared()):
						velocity = ceiling_vertical_velocity + velocity.slide(up_direction);
			
			if(on_floor && floor_stop_on_slope && (velocity.normalized() + up_direction).length() < 0.01):
				var gt:Transform2D = get_global_transform();
				if(result.get_travel().length() <= safe_margin + CMP_EPSILON):
					gt = gt.translated(-result.get_travel());
				set_global_transform(gt);
				velocity = Vector2();
				last_motion = Vector2();
				motion = Vector2();
				break;
			
			if(result.get_remainder().is_zero_approx()):
				motion = Vector2();
				break;
			
			# Move on floor only checks.
			if(floor_block_on_wall && on_wall && motion_slide_up.dot(result.get_normal()) <= 0):
				# Avoid to move forward on a wall if floor_block_on_wall is true.
				if(was_on_floor && !on_floor && !vel_dir_facing_up):
					# If the movement is large the body can be prevented from reaching the walls.
					if(result.get_travel().length() <= safe_margin + CMP_EPSILON):
						# Cancels the motion
						var gt:Transform2D = get_global_transform();
						gt = gt.translated(-result.get_travel());
						set_global_transform(gt);
					# Determines if you are on ground.
					_snap_on_floor(true,false,true);
					velocity = Vector2();
					last_motion = Vector2();
					motion = Vector2();
					break;
				# Prevents the body from being able to climb a slope when it moves forward against the wall.
				elif(!on_floor):
					motion = up_direction * up_direction.dot(result.get_remainder());
					motion = motion.slide(result.get_normal());
				else:
					motion = result.get_remainder();
			# Constant Speed when the slope is upward.
			elif(floor_constant_speed && is_on_floor_only() && can_apply_constant_speed && was_on_floor && motion.dot(result.get_normal()) < 0):
				can_apply_constant_speed = false;
				var motion_slide_norm:Vector2 = result.get_remainder().slide(result.get_normal()).normalized();
				motion = motion_slide_norm * (motion_slide_up.length() - result.get_travel().slide(up_direction).length() - last_travel.slide(up_direction).length());
			# Regular sliding, the last part of the test handle the case when you don't want to slide on the ceiling.
			elif((sliding_enabled || !on_floor) && (!on_ceiling || slide_on_ceiling || !vel_dir_facing_up) && !apply_ceiling_velocity):
				var slide_motion:Vector2 = result.get_remainder().slide(result.get_normal());
				if(slide_motion.dot(velocity) > 0.0):
					motion = slide_motion;
				else:
					motion = Vector2();
				if(slide_on_ceiling):
					# Apply slide only in the direction of the input motion, otherwise just stop to avoid jittering when moving against a wall.
					if(vel_dir_facing_up):
						velocity = velocity.slide(result.get_normal());
					else:
						# Avoid acceleration in slope when falling.
						velocity = up_direction * up_direction.dot(velocity);
			# No sliding on first attempt to keep floor motion stable when possible.
			else:
				motion = result.get_remainder();
				if(on_ceiling && !slide_on_ceiling && vel_dir_facing_up):
					velocity = velocity.slide(up_direction);
					motion = motion.slide(up_direction);
			last_travel = result.get_travel();
		# When you move forward in a downward slope you don’t collide because you will be in the air.
		# This test ensures that constant speed is applied, only if the player is still on the ground after the snap is applied.
		elif(floor_constant_speed && first_slide && _on_floor_if_snapped(was_on_floor,vel_dir_facing_up)):
			can_apply_constant_speed = false;
			sliding_enabled = true;
			var gt:Transform2D = get_global_transform();
			gt.origin = previous_position;
			set_global_transform(gt);
			
			var motion_slide_norm:Vector2 = motion.slide(prev_floor_normal).normalized();
			motion = motion_slide_norm * (motion_slide_up.length());
			collided = true;
		
		can_apply_constant_speed = !can_apply_constant_speed && !sliding_enabled;
		sliding_enabled = true;
		first_slide = false;
		
		if(!collided || motion.is_zero_approx()): break;
	
	_snap_on_floor(was_on_floor,vel_dir_facing_up,false);
	
	# Scales the horizontal velocity according to the wall slope.
	if(is_on_wall_only() && motion_slide_up.dot(motion_results[0].collision_normal) < 0):
		var slide_motion:Vector2 = velocity.slide(motion_results[0].collision_normal);
		if(motion_slide_up.dot(slide_motion) < 0):
			velocity = up_direction * up_direction.dot(velocity);
		else:
			# Keeps the vertical motion from velocity and add the horizontal motion of the projection.
			velocity = up_direction * up_direction.dot(velocity) + slide_motion.slide(up_direction);
	
	# Reset the gravity accumulation when touching the ground.
	if(on_floor && !vel_dir_facing_up): velocity = velocity.slide(up_direction);

func _move_and_slide_floating(delta:float)->void:
	var motion:Vector2 = velocity * delta;
	
	platform_rid = RID();
	platform_object_id = -1;
	floor_normal = Vector2();
	platform_velocity = Vector2();
	
	var first_slide:bool = true;
	for i in max_slides:
		var result:KinematicCollision2D = move_and_collide(motion,false,safe_margin,true);
		var collided:bool = (result is KinematicCollision2D);
		
		last_motion = result.get_travel();
		
		if(collided):
			motion_results.push_back(result);
			_set_collision_direction(result);
			
			if(result.get_remainder().is_zero_approx()):
				motion = Vector2();
				break;
			
			if(wall_min_slide_angle != 0 && result.get_normal().angle_to(-velocity.normalized()) < wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD):
				motion = Vector2();
			elif(first_slide):
				var motion_slide_norm:Vector2 = result.get_remainder().slide(result.get_normal()).normalized();
				motion = motion_slide_norm * (motion.length() - result.get_travel().length());
			else:
				motion = result.remainder.slide(result.get_normal());
			
			if(!collided || motion.is_zero_approx()):
				break;
			
			first_slide = false;

# Method that avoids the p_wall_as_floor parameter for the public method.
func _apply_floor_snap(wall_as_floor:bool)->void:
	if(on_floor): return;
	
	# Snap by at least collision margin to keep floor state consistent.
	var length:float = max(floor_snap_length,safe_margin);
	
	var gt:Transform2D = get_global_transform();
	var result:KinematicCollision2D = move_and_collide(-up_direction * length,true,safe_margin,true);
	if(result is KinematicCollision2D):
		if((result.get_normal().angle_to(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) ||
		(wall_as_floor && result.angle_to(-up_direction) > floor_max_angle + FLOOR_ANGLE_THRESHOLD)):
			on_floor = true;
			floor_normal = result.get_normal();
			_set_platform_data(result);
			var new_travel:Vector2 = result.get_travel();
			if(floor_stop_on_slope):
				# move and collide may stray the object a bit because of pre un-stucking,
				# so only ensure that motion happens on floor direction in this case.
				if(new_travel.length() > safe_margin):
					new_travel = up_direction * up_direction.dot(new_travel);
				else:
					new_travel = Vector2();
			
			gt = gt.translated(new_travel);
			set_global_transform(gt);

func _snap_on_floor(was_on_floor:bool,vel_dir_facing_up:bool,wall_as_floor:bool)->void:
	if(on_floor || !was_on_floor || vel_dir_facing_up): return;
	_apply_floor_snap(wall_as_floor);

func _on_floor_if_snapped(was_on_floor:bool,vel_dir_facing_up:bool)->bool:
	if(up_direction == Vector2() || on_floor || !was_on_floor || vel_dir_facing_up): return false;
	
	# Snap by at least collision margin to keep floor state consistent.
	var length:float = max(floor_snap_length, safe_margin);
	
	var result:KinematicCollision2D = move_and_collide(-up_direction * length,true,safe_margin,true);
	if(result is KinematicCollision2D):
		if(result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
			return true;
	
	return false;

func _set_collision_direction(result:KinematicCollision2D)->void:
	if(motion_mode == MOTION_MODE_GROUNDED && result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
		on_floor = true;
		floor_normal = result.get_normal();
		_set_platform_data(result);
	elif(motion_mode == MOTION_MODE_GROUNDED && result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
		on_ceiling = true;
	else:
		on_wall = true;
		wall_normal = result.get_normal();
		# Don't apply wall velocity when the collider is a CharacterBody2D.
		if(result.get_collider() is CharacterBody2D):
			_set_platform_data(result);

func _set_platform_data(result:KinematicCollision2D)->void:
	platform_rid = result.get_collider_rid();
	platform_object_id = result.get_collider_id();
	platform_velocity = result.get_collider_velocity();
	platform_layer = PhysicsServer2D.body_get_collision_layer(platform_rid);

func is_on_floor()->bool: return on_floor;
func is_on_floor_only()->bool: return on_floor && !on_wall && !on_ceiling;
func is_on_wall()->bool: return on_wall;
func is_on_wall_only()->bool: return on_wall && !on_floor && !on_ceiling;
func is_on_ceiling()->bool: return on_ceiling;
func is_on_ceiling_only()->bool: return on_ceiling && !on_floor && !on_wall;

func get_floor_normal()->Vector2: return floor_normal;
func get_wall_normal()->Vector2: return wall_normal;

func get_last_motion()->Vector2: return last_motion;
func get_position_delta()->Vector2: return get_global_transform().origin - previous_position;

func get_real_velocity()->Vector2: return real_velocity;
func get_platform_velocity()->Vector2: return platform_velocity;

func get_slide_collision_count()->int: return motion_results.size();
func get_slide_collision(bounce:int)->KinematicCollision2D: return motion_results[bounce];
func get_last_slide_collision()->KinematicCollision2D:
	if(motion_results.size() == 0): return KinematicCollision2D.new();
	return get_slide_collision(motion_results.size() - 1);

func set_safe_margin(margin:float)->void: safe_margin = margin;
func get_safe_margin()->float: return safe_margin;
