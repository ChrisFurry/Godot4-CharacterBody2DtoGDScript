# We can't extend PhysicsBody2D, and AnimatableBody2D does not already define variables we need.
extends AnimatableBody2D

@export var motion_mode:CharacterBody2D.MotionMode = CharacterBody2D.MotionMode.MOTION_MODE_GROUNDED
@export var up_direction:Vector2	= Vector2.UP:
	set(value): set_up_direction(value)
@export var slide_on_ceiling:bool = true
@export var wall_min_slide_angle:float = deg_to_rad(15.0)

@export_group("Floor","floor_")

@export var floor_constant_speed:bool = false:
	set(value): set_floor_constant_speed_enabled(value)
@export var floor_stop_on_slope:bool = true:
	set(value): set_floor_stop_on_slope_enabled(value)
@export var floor_block_on_wall:bool = true:
	set(value): set_floor_block_on_wall_enabled(value)

@export var floor_max_angle:float	= deg_to_rad(45.0)
@export var floor_snap_length:float = 8.0:
	set(value): set_floor_snap_length(value)

@export_group("Moving Platform","platform_")

@export var platform_on_leave:CharacterBody2D.PlatformOnLeave = CharacterBody2D.PlatformOnLeave.PLATFORM_ON_LEAVE_ADD_VELOCITY
@export_flags_2d_physics var platform_floor_layers:int = (1 << 32) # UINT32_MAX
@export_flags_2d_physics var platform_wall_layers = 0

@export_group("Collision")

@export var safe_margin:float = 0.08

var max_slides:int = 4:
	set(value): set_max_slides(value)
var platform_layer:int = 0

var velocity:Vector2 = Vector2.ZERO

var floor_normal:Vector2
var platform_velocity:Vector2
var wall_normal:Vector2
var last_motion:Vector2
var previous_position:Vector2
var real_velocity:Vector2

var platform:Node
var platform_rid:RID

var on_floor:bool = false
var on_ceiling:bool = false
var on_wall:bool = false

var motion_results:Array[KinematicCollision2D]
var slide_colliders:Array[KinematicCollision2D]

# Constants

const FLOOR_ANGLE_THRESHOLD = 0.01

const CMP_EPSILON = 0.001

# Base Functions
# NOTE: These are what you want.

func _notification(what:int)->void:
	match(what):
		NOTIFICATION_ENTER_TREE:
			on_floor = false
			platform_rid = RID()
			platform = null
			on_ceiling = false
			on_wall = false
			motion_results.clear()
			platform_velocity = Vector2()

func move_and_slide()->bool:
	var delta:float = get_physics_process_delta_time() if(Engine.is_in_physics_frame())else get_process_delta_time()
	
	var current_platform_velocity:Vector2 = platform_velocity
	var previous_transform:Transform2D = global_transform
	previous_position = previous_transform.get_origin()
	# Platform Movement
	if((on_floor or on_wall) and platform_rid.is_valid()):
		var excluded:bool = false
		if(on_floor):
			excluded = (platform_floor_layers & platform_layer) == 0
		elif(on_wall):
			excluded = (platform_wall_layers & platform_layer) == 0
		if(not excluded):
			var body_state:PhysicsDirectBodyState2D = PhysicsServer2D.body_get_direct_state(platform_rid)
			if(body_state):
				var local_position = previous_transform.get_origin() - body_state.transform.get_origin()
				current_platform_velocity = body_state.get_velocity_at_local_position(local_position)
			else:
				current_platform_velocity = Vector2()
				platform_rid = RID()
				platform = null
		else:
			current_platform_velocity = Vector2()
	motion_results.clear()
	last_motion = Vector2()
	
	var was_on_floor:bool = on_floor
	on_floor = false
	on_ceiling = false
	on_wall = false
	
	if(not current_platform_velocity.is_zero_approx()):
		if(platform_rid.is_valid()):
			add_collision_exception_with(platform)
			var result = move_and_collide(current_platform_velocity * delta,false,safe_margin,true)
			if(result):
				motion_results.push_back(result)
				_set_collision_direction(result)
			remove_collision_exception_with(platform)
	
	if(motion_mode == CharacterBody2D.MotionMode.MOTION_MODE_GROUNDED):
		_move_and_slide_grounded(delta,was_on_floor)
	else:
		_move_and_slide_floating(delta)
	
	real_velocity = get_position_delta() / delta
	
	if(platform_on_leave != CharacterBody2D.PlatformOnLeave.PLATFORM_ON_LEAVE_DO_NOTHING):
		if(not on_floor and not on_wall):
			if(platform_on_leave == CharacterBody2D.PlatformOnLeave.PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY and current_platform_velocity.dot(up_direction) < 0.0):
				current_platform_velocity = current_platform_velocity.slide(up_direction)
			velocity += current_platform_velocity
	
	return motion_results.size() > 0

func _move_and_slide_grounded(delta:float,was_on_floor:bool)->void:
	var motion:Vector2 = velocity * delta
	var motion_slide_up:Vector2 = motion.slide(up_direction)
	
	var prev_floor_normal:Vector2 = floor_normal
	
	platform_rid = RID()
	platform = null
	floor_normal = Vector2()
	platform_velocity = Vector2()
	
	var sliding_enabled:bool = not floor_stop_on_slope
	var can_apply_constant_speed:bool = sliding_enabled
	var apply_ceiling_velocity:bool = false
	var first_slide:bool = true
	var vel_dir_facing_up:bool = velocity.dot(up_direction) > 0
	var last_travel:Vector2
	
	for i in max_slides:
		var prev_position:Vector2 = global_transform.origin
		var result:KinematicCollision2D = _move_and_collide_cancel_sliding(motion,false,safe_margin,true,not sliding_enabled)
		var collided:bool = result != null
		if(collided):
			last_motion = result.get_travel()
			
			motion_results.push_back(result)
			_set_collision_direction(result)
			
			if(on_ceiling and not result.get_collider_velocity().is_zero_approx() and result.get_collider_velocity().dot(up_direction) < 0.0):
				if(not slide_on_ceiling or motion.dot(up_direction) < 0.0 or (result.get_normal() + up_direction).length() < 0.01):
					apply_ceiling_velocity = true
					var ceiling_vert_velocity:Vector2 = up_direction * up_direction.dot(result.get_collider_velocity())
					var motion_vert_velocity:Vector2 = up_direction * up_direction.dot(velocity)
					if(motion_vert_velocity.dot(up_direction) > 0.0 or ceiling_vert_velocity.length_squared() > motion_vert_velocity.length_squared()):
						velocity = ceiling_vert_velocity + velocity.slide(up_direction)
			
			if(on_floor and floor_stop_on_slope and (velocity.normalized() + up_direction).length() < 0.01):
				if(result.get_travel().length() <= safe_margin + CMP_EPSILON):
					global_transform.origin -= result.get_travel()
				velocity = Vector2()
				last_motion = Vector2()
				motion = Vector2()
				break
			
			if(result.get_remainder().is_zero_approx()):
				motion = Vector2()
				break
			
			if(floor_block_on_wall and on_wall and motion_slide_up.dot(result.get_normal()) <= 0.0):
				if(was_on_floor and not on_floor and not vel_dir_facing_up):
					if(result.get_travel().length() <= safe_margin + CMP_EPSILON):
						global_transform.origin -= result.get_travel()
					_snap_on_floor(true,false,true)
					velocity = Vector2()
					last_motion = Vector2()
					motion = Vector2()
					break
				elif(not on_floor):
					motion = up_direction * up_direction.dot(result.get_remainder())
					motion = motion.slide(result.get_normal())
				else:
					motion = result.get_remainder()
			elif(floor_constant_speed and is_on_floor_only() and can_apply_constant_speed and was_on_floor and motion.dot(result.get_normal()) < 0.0):
				can_apply_constant_speed = false
				var motion_slide_norm:Vector2 = result.get_remainder().slide(result.get_normal()).normalized()
				motion = motion_slide_norm * (motion_slide_up.length() - result.get_travel().slide(up_direction).length() - last_travel.slide(up_direction).length())
			elif((sliding_enabled or not on_floor) and (not on_ceiling or slide_on_ceiling or not vel_dir_facing_up) and not apply_ceiling_velocity):
				var slide_motion:Vector2 = result.get_remainder().slide(result.get_normal())
				if(slide_motion.dot(velocity) > 0.0):
					motion = slide_motion
				else:
					motion = Vector2()
				
				if(slide_on_ceiling and on_ceiling):
					if(vel_dir_facing_up):
						velocity = velocity.slide(result.get_normal())
					else:
						velocity = up_direction * up_direction.dot(velocity)
			else:
				motion = result.get_remainder()
				if(on_ceiling and not slide_on_ceiling and vel_dir_facing_up):
					velocity = velocity.slide(up_direction)
					motion = motion.slide(up_direction)
			
			last_travel = result.get_travel()
		elif(floor_constant_speed and first_slide and _on_floor_if_snapped(was_on_floor, vel_dir_facing_up)):
			can_apply_constant_speed = false
			sliding_enabled = true
			global_transform.origin = prev_position
			
			var motion_slide_norm:Vector2 = motion.slide(prev_floor_normal).normalized()
			motion = motion_slide_norm * (motion_slide_up.length())
			collided = true
		
		can_apply_constant_speed = not can_apply_constant_speed and not sliding_enabled
		sliding_enabled = true
		first_slide = false
		
		if(not collided or motion.is_zero_approx()): break
	
	_snap_on_floor(was_on_floor,vel_dir_facing_up,false)
	
	if(is_on_wall_only() and motion_slide_up.dot(motion_results.get(0).get_normal()) < 0.0):
		var slide_motion:Vector2 = velocity.slide(motion_results.get(0).get_normal())
		if(motion_slide_up.dot(slide_motion) < 0.0):
			velocity = up_direction * up_direction.dot(velocity)
		else:
			velocity = up_direction * up_direction.dot(velocity) + slide_motion.slide(up_direction)
	
	if(on_floor and not vel_dir_facing_up): velocity = velocity.slide(up_direction)

func _move_and_slide_floating(delta:float)->void:
	var motion:Vector2 = velocity * delta
	
	platform_rid = RID()
	platform = null
	floor_normal = Vector2()
	platform_velocity = Vector2()
	
	var first_slide:bool = true
	for i in max_slides:
		var result:KinematicCollision2D = move_and_collide(motion,false,safe_margin,false)
		
		last_motion = result.get_travel() if(result)else Vector2()
		
		if(result):
			motion_results.push_back(result)
			
			if(result.get_remainder().is_zero_approx()):
				motion = Vector2()
				break
			
			if(wall_min_slide_angle != 0 and result.get_angle(-velocity.normalized()) < wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD):
				motion = Vector2()
			elif(first_slide):
				var motion_norm:Vector2 = result.get_remainder().slide(result.get_normal()).normalized()
				motion = motion_norm * (motion.length() - result.get_travel().length())
			else:
				motion = result.get_remainder().slide(result.get_normal())
			
			if(motion.dot(velocity) <= 0.0): motion = Vector2()
		
		if(not result or motion.is_zero_approx()): break
		
		first_slide = false

func apply_floor_snap()->void:
	_apply_floor_snap(false)

func _apply_floor_snap(wall_as_floor:bool)->void:
	if(on_floor): return
	
	var length:float = max(floor_snap_length,safe_margin)
	
	var result:KinematicCollision2D = move_and_collide(-up_direction * length,true,safe_margin,true)
	if(result):
		if((result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) or
				(wall_as_floor and result.get_angle(-up_direction) > floor_max_angle + FLOOR_ANGLE_THRESHOLD)):
			on_floor = true
			floor_normal = result.get_normal()
			_set_platform_data(result)
			
			var new_travel = result.get_travel()
			if(new_travel.length() > safe_margin):
				new_travel = up_direction * up_direction.dot(result.get_normal())
			else:
				new_travel = Vector2()
			
			global_transform.origin += new_travel

func _snap_on_floor(was_on_floor:bool,vel_dir_facing_up:bool,wall_as_floor:bool)->void:
	if(on_floor or not was_on_floor or vel_dir_facing_up): return
	
	_apply_floor_snap(wall_as_floor)

func _on_floor_if_snapped(was_on_floor:bool,vel_dir_facing_up:bool)->bool:
	if(up_direction.is_zero_approx() or on_floor or not was_on_floor or vel_dir_facing_up): return false
	
	var length:float = max(floor_snap_length,safe_margin)
	
	var result:KinematicCollision2D = move_and_collide(-up_direction * length,true,safe_margin,true)
	if(result):
		if(result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
			return true
	
	return false

func _set_collision_direction(result:KinematicCollision2D)->void:
	if(motion_mode == CharacterBody2D.MOTION_MODE_GROUNDED and result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
		on_floor = true
		floor_normal = result.get_normal()
		_set_platform_data(result)
	elif(motion_mode == CharacterBody2D.MOTION_MODE_GROUNDED and result.get_angle(-up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD):
		on_ceiling = true
	else:
		on_wall = true
		wall_normal = result.get_normal()
		if(not (result.get_collider() is CharacterBody2D)):
			_set_platform_data(result)

func _set_platform_data(result:KinematicCollision2D)->void:
	var bs:PhysicsDirectBodyState2D = PhysicsServer2D.body_get_direct_state(result.get_collider_rid())
	if(not bs): return
	
	platform_rid = result.get_collider_rid()
	platform = result.get_collider()
	platform_velocity = result.get_collider_velocity()
	platform_layer = bs.collision_layer

# Get & Set Data
# NOTE: I didn't have to port these, but I chose to.

func get_velocity()->Vector2: return velocity
func set_velocity(new:Vector2)->void: velocity = new

func is_on_floor()->bool: return on_floor
func is_on_floor_only()->bool: return on_floor and not on_wall and not on_ceiling
func is_on_wall()->bool: return on_wall
func is_on_wall_only()->bool: return on_wall and not on_floor and not on_ceiling
func is_on_ceiling()->bool: return on_ceiling
func is_on_ceiling_only()->bool: return on_ceiling and not on_floor and not on_wall

func get_floor_normal()->Vector2: return floor_normal
func get_wall_normal()->Vector2: return wall_normal

func get_last_motion()->Vector2: return last_motion

func get_position_delta()->Vector2: return get_global_transform().get_origin() - previous_position

func get_real_velocity()->Vector2: return real_velocity

func get_floor_angle(p_up_direction:Vector2)->float: 
	assert(not p_up_direction.is_zero_approx())
	return acos(floor_normal.dot(p_up_direction))

func get_platform_velocity()->Vector2: return platform_velocity

func get_slide_collision_count()->int: return motion_results.size()

func get_slide_collision(bounce:int)->KinematicCollision2D: 
	assert(bounce >= 0 and bounce < motion_results.size())
	return motion_results[bounce]

func _get_last_slide_collision()->KinematicCollision2D: 
	if(motion_results.is_empty()): return KinematicCollision2D.new()
	return motion_results[motion_results.size()-1]

func set_safe_margin(margin:float)->void: safe_margin = margin
func get_safe_margin()->float: return safe_margin

func is_floor_stop_on_slope_enabled()->bool: return floor_stop_on_slope
func set_floor_stop_on_slope_enabled(enabled:bool)->void: floor_stop_on_slope = enabled

func is_floor_constant_speed_enabled()->bool: return floor_constant_speed
func set_floor_constant_speed_enabled(enabled:bool)->void: floor_constant_speed = enabled

func is_floor_block_on_wall_enabled()->bool: return floor_block_on_wall
func set_floor_block_on_wall_enabled(enabled:bool)->void: floor_block_on_wall = enabled

func is_slide_on_ceiling_enabled()->bool: return slide_on_ceiling
func set_slide_on_ceiling_enabled(enabled:bool)->void: slide_on_ceiling = enabled

func get_platform_floor_layer()->int: return platform_floor_layers
func set_platform_floor_layers(excluded_layers:int)->void: platform_floor_layers = excluded_layers

func get_platform_wall_layer()->int: return platform_wall_layers
func set_platform_wall_layers(excluded_layers:int)->void: platform_wall_layers = excluded_layers

func set_motion_mode(mode:CharacterBody2D.MotionMode)->void: motion_mode = mode
func get_motion_mode()->CharacterBody2D.MotionMode: return motion_mode

func set_platform_on_leave(on_leave:CharacterBody2D.PlatformOnLeave)->void: platform_on_leave = on_leave
func get_platform_on_leave()->CharacterBody2D.PlatformOnLeave: return platform_on_leave

func get_max_slides()->int: return max_slides
func set_max_slides(p_max_slides:int)->void:
	assert(p_max_slides > 0)
	max_slides = p_max_slides

func get_floor_max_angle()->float: return floor_max_angle
func set_floor_max_angle(radians:float)->void: floor_max_angle = radians

func get_floor_snap_length()->float: return floor_snap_length
func set_floor_snap_length(length:float)->void: 
	assert(floor_snap_length >= 0.0)
	floor_snap_length = length

func get_wall_min_slide_angle()->float: return wall_min_slide_angle
func set_wall_min_slide_angle(radians:float)->void: wall_min_slide_angle = radians

func get_up_direction()->Vector2: return up_direction
func set_up_direction(p_up_direction:Vector2)->void:
	assert(not p_up_direction.is_zero_approx())
	up_direction = p_up_direction

## Internally the move_and_collide function is always called from gdscript without "cancel sliding", and there is no way to enable this behavior, other than this.
func _move_and_collide_cancel_sliding(motion:Vector2,test_only:bool,margin:float,recovery_as_collision:bool,cancel_sliding:bool)->KinematicCollision2D:
	var params:PhysicsTestMotionParameters2D = PhysicsTestMotionParameters2D.new()
	params.from = global_transform
	params.motion = motion
	params.margin = safe_margin
	params.recovery_as_collision = recovery_as_collision
	var result:PhysicsTestMotionResult2D = PhysicsTestMotionResult2D.new()
	var colliding:bool = PhysicsServer2D.body_test_motion(get_rid(),params,result)
	
	var true_travel:Vector2 = result.get_travel()
	# unused, since you can't set the travel and remainder via gdscript.
	#var true_remainder:Vector2 = result.get_remainder()
	if(cancel_sliding):
		var motion_length:float = motion.length()
		var precision:float = 0.001
		
		if(colliding):
			precision += motion_length * (result.get_collision_unsafe_fraction() - result.get_collision_safe_fraction())
			
			if(result.get_collision_depth() > margin + precision): cancel_sliding = false
		
		if(cancel_sliding):
			var motion_normal:Vector2
			if(motion_length > 0.001): motion_normal = motion.normalized()
			
			var projected_length:float = result.get_travel().dot(motion_normal)
			var recovery:Vector2 = result.get_travel() - motion_normal * projected_length
			var recovery_length:float = recovery.length()
			
			if(recovery_length < margin + precision):
				true_travel = motion_normal * projected_length
				#true_remainder = motion - true_travel
	var new_result = null
	
	if(colliding): # Hack since we can't add PhysicsTestMotionResult2D to our own KinematicCollision2D (I believe this is an oversight)
		new_result = move_and_collide(motion,true,margin,recovery_as_collision)
	
	if(not test_only):
		global_transform.origin += true_travel
	
	return new_result if(colliding)else null
