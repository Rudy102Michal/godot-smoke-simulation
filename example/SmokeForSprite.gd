extends Sprite

var smoke : Smoke = null
var smoke_size : float = 200.0
var m_last_pos : Vector2


func _ready():
	var image_texture = ImageTexture.new()
	m_last_pos = Vector2(-1, -1)
	image_texture.create( smoke_size, smoke_size ,Image.FORMAT_L8 )
	smoke = Smoke.new()

	smoke.init( smoke_size, image_texture )
	
#	smoke.attach_density_source(100, 100, 16, 1);
	
#	smoke.attach_velocity_source(88.5, 110.5, 1.0, 0.0, 100)

	smoke.set_diffusion_rate(0.001)
	smoke.set_viscosity(0.0001)
	smoke.set_relaxation_iteration_count(10)

	self.set_texture( smoke.get_texture() )
#	set_process( true )

func _process(delta):
	
	# "click2" action is right mouse button click
	if Input.is_action_just_pressed("click_2"):
		var mp = get_viewport().get_mouse_position()
		for i in range(-5, 5):
			for j in range(-5, 5):
				smoke.add_density(int(mp.x) + i, int(mp.y) + j, 1.0)
	
	# "click" action is left mouse button click
	if Input.is_action_pressed("click"):
		var m_pos = get_viewport().get_mouse_position()
		if m_last_pos.x > 0:
			var m_change : Vector2 = m_pos - m_last_pos
			m_change *= 2.0;
			for i in range(5):
				var x = floor(m_pos.x + rand_range(-2, 3))
				var y = floor(m_pos.y + rand_range(-2, 3))
				smoke.add_velocity(x, y, m_change.x, m_change.y)
		m_last_pos = m_pos
	else:
		m_last_pos.x = -1.0
	
	smoke.update(delta)
