import math

servo_param = {
	1: {
		'home_encoder': 0x200,
        'max_encoder': 0x3FF,
        'rad_per_enc': math.radians(300.0) / 0xFFF, 
        'max_ang': math.radians(100),
        'min_ang': math.radians(-10),
        'flipped': False,
        'max_speed': math.radians(130) / 2.0
	}
}
