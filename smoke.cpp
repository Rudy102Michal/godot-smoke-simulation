/*****************************************
*	Implementation by M. Gornicki
*	Hugely based on work of Jos Stam
*	and explanation by Mike Ash 
******************************************/
#include "smoke.h"

#define GIndex(i, j, N) ((i) + (N) * (j))

#define ix(i, j) ((i) + (N) * (j))

Smoke::~Smoke() {

	memdelete(d_sources);
	memdelete(v_sources);

	if (d) {
		memdelete_arr(d);
		d = nullptr;
	}

	if (d0) {
		memdelete_arr(d0);
		d0 = nullptr;
	}

	if (u) {
		memdelete_arr(u);
		u = nullptr;
	}

	if (u0) {
		memdelete_arr(u0);
		u0 = nullptr;
	}

	if (v) {
		memdelete_arr(v);
		v = nullptr;
	}

	if (v0) {
		memdelete_arr(v0);
		v0 = nullptr;
	}
}

Smoke::Smoke() :
		d(nullptr),
		d0(nullptr),
		u(nullptr),
		u0(nullptr),
		v(nullptr),
		v0(nullptr) {
}

void Smoke::init(int size, const Ref<ImageTexture> &p_texture) {
	set_texture(p_texture);
	gauss_seidel_it_count = 20;
	N = size;

	buffer1 = memnew(PoolVector<uint8_t>);

	d = memnew_arr(float_t, N * N);
	d0 = memnew_arr(float_t, N * N);
	u = memnew_arr(float_t, N * N);
	u0 = memnew_arr(float_t, N * N);
	v = memnew_arr(float_t, N * N);
	v0 = memnew_arr(float_t, N * N);

	for (int i = 0; i < N * N; ++i) {
		d[i] = d0[i] = u[i] = u0[i] = v[i] = v0[i] = 0.0f;
	}

	d_sources = memnew(PoolVector<SmokeSource>);
	d_sources->resize(0);

	v_sources = memnew(PoolVector<VelSource>);
	v_sources->resize(0);

	buffer1->resize(N * N);

	PoolVector<uint8_t>::Write write = buffer1->write();
	for (int i = 0; i < N * N; i++) {
		write[i] = 255;
	}
	write = PoolVector<uint8_t>::Write();
	image->create(N, N, false, Image::FORMAT_L8, *buffer1);
	texture_ref->create_from_image(image, texture_ref->FLAG_VIDEO_SURFACE);
}

void Smoke::update(float dt) {

	// Uncomment the lines if you want to have some static velocity/density sources
	// apply sources

	//_apply_density_sources(dt);
	//_apply_velocity_sources(dt);

	// diffuse velocity
	_diffuse(u0, u, viscosity, dt, BND_VER_NEGATE);
	_diffuse(v0, v, viscosity, dt, BND_HOR_NEGATE);

	// project it (mass conservation)
	_project(u0, v0, u, v);

	// advect velocity
	_advect(u, u0, u0, v0, dt, BND_VER_NEGATE);
	_advect(v, v0, u0, v0, dt, BND_HOR_NEGATE);

	// project it
	_project(u, v, u0, v0);

	// diffuse density
	_diffuse(d0, d, diffusion_rate, dt, BND_MISC);
	// advect density
	_advect(d, d0, u, v, dt, BND_MISC);

	// swapping happens on it's own due to the way the pointers are being passed to the functions

	PoolVector<uint8_t>::Write buffer_write = buffer1->write();

	for (int i = 0; i < buffer1->size(); ++i) {
		buffer_write[i] = static_cast<uint8_t>(MIN(1.0f, d[i]) * 255.0f);
	}

	buffer_write = PoolVector<uint8_t>::Write();

	image->create(N, N, false, Image::FORMAT_L8, *buffer1);
	texture_ref->set_data(image);
}

void Smoke::set_relaxation_iteration_count(int it_count) {
	gauss_seidel_it_count = static_cast<size_t>(it_count);
}

/*
void Smoke::density_diffuse(float delta_time) {

	float diff_rate = delta_time * diffusion_rate * static_cast<float>(width * height);
	float diff_rate_divider = 1.f / (1.f + 4.0 * diff_rate);
	PoolVector<float_t>::Read prev_dens_read = prev_density->read();
	PoolVector<float_t>::Read dens_read = density->read();
	PoolVector<float_t>::Write dens_write = density->write();

	for (int k = 0; k < gauss_seidel_it_count; ++k) {
		for (int i = 1; i < width - 1; ++i) {
			for (int j = 1; j < height - 1; ++j) {
				dens_write[GIndex(i, j, width)] = (prev_dens_read[GIndex(i, j, width)] + diff_rate * (dens_read[GIndex(i - 1, j, width)] +
																											 dens_read[GIndex(i + 1, j, width)] +
																											 dens_read[GIndex(i, j - 1, width)] +
																											 dens_read[GIndex(i, j + 1, width)])) *
												  diff_rate_divider;
			}
		}
	}

	// Boundaries!!!
}

void Smoke::density_move(float delta_time) {

	float width_f = static_cast<float>(width);
	float height_f = static_cast<float>(height);
	float dt = (width_f < height_f) ? static_cast<float>(width_f) : static_cast<float>(height_f);
	dt *= delta_time;

	PoolVector<float_t>::Read x_vel_read = velocity_x->read();
	PoolVector<float_t>::Read y_vel_read = velocity_y->read();

	PoolVector<float_t>::Write dens_write = density->write();
	PoolVector<float_t>::Read prev_dens_read = prev_density->read();

	float s0, s1, t0, t1;

	for (int i = 1; i < width - 1; ++i) {
		for (int j = 0; j < height - 1; ++j) {
			float x_ = static_cast<float>(i) - dt * x_vel_read[GIndex(i, j, width)];
			float y_ = static_cast<float>(j) - dt * y_vel_read[GIndex(i, j, width)];

			// CLAMP THAT
			// x_ = (x_ < 0.5f) ? 0.5f : x_;
			if (x_ < 0.5f)
				x_ = 0.5f;
			else if (x_ > width_f + 0.5f)
				x_ = width_f + 0.5f;

			if (y_ < 0.5f)
				y_ = 0.5f;
			else if (y_ > height_f + 0.5f)
				y_ = height_f + 0.5f;

			int i_prev = static_cast<int>(x_);
			int j_prev = static_cast<int>(y_);

			s1 = x_ - static_cast<float>(i_prev);
			s0 = 1.0f - s1;
			t1 = y_ - static_cast<float>(j_prev);
			t0 = 1.0f - t1;

			dens_write[GIndex(i, j, width)] = s0 * (t0 * prev_dens_read[GIndex(i_prev, j_prev, width)] + t1 * prev_dens_read[GIndex(i_prev, j_prev + 1, width)]) + s1 * (t0 * prev_dens_read[GIndex(i_prev + 1, j_prev, width)] + t1 * prev_dens_read[GIndex(i_prev + 1, j_prev + 1, width)]);
		}
	}
}

void Smoke::density_step(float delta_time) {
	BUFFER_SWAP(prev_density, density);
	density_diffuse(delta_time);
	BUFFER_SWAP(prev_density, density);
	universal_advection(delta_time, density, prev_density, velocity_x, velocity_y);
}

void Smoke::universal_advection(float delta_time, PoolVector<float_t> *base, PoolVector<float_t> *base_prev, PoolVector<float_t> *vel_x, PoolVector<float_t> *vel_y) {
	float width_f = static_cast<float>(width);
	float height_f = static_cast<float>(height);
	float dt = (width_f < height_f) ? static_cast<float>(width_f) : static_cast<float>(height_f);
	dt *= delta_time;

	PoolVector<float_t>::Read x_vel_read = vel_x->read();
	PoolVector<float_t>::Read y_vel_read = vel_y->read();

	PoolVector<float_t>::Write base_write = base->write();
	PoolVector<float_t>::Read base_prev_read = base_prev->read();

	float s0, s1, t0, t1;

	for (int i = 1; i < width - 1; ++i) {
		for (int j = 0; j < height - 1; ++j) {
			float x_ = static_cast<float>(i) - dt * x_vel_read[GIndex(i, j, width)];
			float y_ = static_cast<float>(j) - dt * y_vel_read[GIndex(i, j, width)];

			// CLAMP THAT
			// x_ = (x_ < 0.5f) ? 0.5f : x_;
			if (x_ < 0.5f)
				x_ = 0.5f;
			else if (x_ > width_f + 0.5f)
				x_ = width_f + 0.5f;

			if (y_ < 0.5f)
				y_ = 0.5f;
			else if (y_ > height_f + 0.5f)
				y_ = height_f + 0.5f;

			int i_prev = static_cast<int>(x_);
			int j_prev = static_cast<int>(y_);

			s1 = x_ - static_cast<float>(i_prev);
			s0 = 1.0f - s1;
			t1 = y_ - static_cast<float>(j_prev);
			t0 = 1.0f - t1;

			base_write[GIndex(i, j, width)] = s0 * (t0 * base_prev_read[GIndex(i_prev, j_prev, width)] + t1 * base_prev_read[GIndex(i_prev, j_prev + 1, width)]) + s1 * (t0 * base_prev_read[GIndex(i_prev + 1, j_prev, width)] + t1 * base_prev_read[GIndex(i_prev + 1, j_prev + 1, width)]);
		}
	}
}

void Smoke::universal_diffusion(float delta_time, PoolVector<float_t> *base, PoolVector<float_t> *base_prev) {

	float diff_rate = delta_time * diffusion_rate * static_cast<float>(width * height);
	float diff_rate_divider = 1.f / (1.f + 4.0 * diff_rate);
	PoolVector<float_t>::Read prev_base_read = base_prev->read();
	PoolVector<float_t>::Read base_read = base->read();
	PoolVector<float_t>::Write base_write = base->write();

	for (int k = 0; k < gauss_seidel_it_count; ++k) {
		for (int i = 1; i < width - 1; ++i) {
			for (int j = 1; j < height - 1; ++j) {
				base_write[GIndex(i, j, width)] = (prev_base_read[GIndex(i, j, width)] + diff_rate * (base_read[GIndex(i - 1, j, width)] +
																											 base_read[GIndex(i + 1, j, width)] +
																											 base_read[GIndex(i, j - 1, width)] +
																											 base_read[GIndex(i, j + 1, width)])) *
												  diff_rate_divider;
			}
		}
	}
}

void Smoke::velocity_step(float delta_time) {

	BUFFER_SWAP(prev_velocity_x, velocity_x);
	universal_diffusion(delta_time, velocity_x, prev_velocity_x);
	fill_boundaries(velocity_x, 1.f);
	BUFFER_SWAP(prev_velocity_y, velocity_y);
	universal_diffusion(delta_time, velocity_y, prev_velocity_y);
	fill_boundaries(velocity_y, 2.f);
	velocity_projection(delta_time);
	BUFFER_SWAP(prev_velocity_y, velocity_y);
	BUFFER_SWAP(prev_velocity_x, velocity_x);
	universal_advection(delta_time, velocity_x, prev_velocity_x, prev_velocity_x, prev_velocity_y);
	fill_boundaries(velocity_x, 1.f);
	universal_advection(delta_time, velocity_y, prev_velocity_y, prev_velocity_x, prev_velocity_y);
	fill_boundaries(velocity_y, 2.f);
}

void Smoke::velocity_projection(float delta_time) {

	float h_x = 1.0f / static_cast<float>(width);
	float h_y = 1.0f / static_cast<float>(height);

	float *divergence = new float[width * height];
	float *pressure = new float[width * height];

	PoolVector<float_t>::Read x_vel_read = velocity_x->read();
	PoolVector<float_t>::Read y_vel_read = velocity_y->read();

	PoolVector<float_t>::Write x_vel_write = velocity_x->write();
	PoolVector<float_t>::Write y_vel_write = velocity_y->write();

	for (int i = 1; i < width - 1; ++i) {
		for (int j = 1; j < width - 1; ++j) {
			divergence[GIndex(i, j, width)] = -0.5f * (h_x * (x_vel_read[GIndex(i + 1, j, width)] - x_vel_read[GIndex(i + 1, j, width)]) + h_y * (y_vel_read[GIndex(i, j - 1, width)] - y_vel_read[GIndex(i, j + 1, width)]));
			pressure[GIndex(i, j, width)] = 0.0f;
		}
	}

	fill_boundaries(divergence, 0);
	fill_boundaries(pressure, 0);

	for (int k = 0; k < gauss_seidel_it_count; ++k) {
		for (int i = 0; i < width - 1; ++i) {
			for (int j = 0; j < height - 1; ++j) {
				pressure[GIndex(i, j, width)] = 0.25f * (divergence[GIndex(i, j, width)] +
																pressure[GIndex(i - 1, j, width)] +
																pressure[GIndex(i + 1, j, width)] +
																pressure[GIndex(i, j - 1, width)] +
																pressure[GIndex(i, j + 1, width)]);
			}
		}
	}

	fill_boundaries(pressure, 0);

	for (int i = 1; i < width - 1; ++i) {
		for (int j = 0; j < height - 1; ++j) {
			x_vel_write[GIndex(i, j, width)] -= 0.5f * (pressure[GIndex(i + 1, j, width)] - pressure[GIndex(i - 1, j, width)]) / h_x;
			y_vel_write[GIndex(i, j, width)] -= 0.5f * (pressure[GIndex(i, j + 1, width)] - pressure[GIndex(i, j - 1, width)]) / h_y;
		}
	}

	fill_boundaries(velocity_x, 1.0);
	fill_boundaries(velocity_y, 2.0);

	delete[] divergence;
	delete[] pressure;
}

void Smoke::buffers_swap(PoolVector<float_t> *a, PoolVector<float_t> *b) {
	PoolVector<float_t> *tmp = a;
}

void Smoke::fill_boundaries(PoolVector<float_t> *a, float fill_value) {

	PoolVector<float_t>::Write a_write = a->write();
	PoolVector<float_t>::Read a_read = a->read();

	for (int i = 1; i < width - 1; ++i) {
		a_write[GIndex(i, 0, width)] = (fill_value == 2) ? -a_read[GIndex(i, 1, width)] : a_read[GIndex(i, 1, width)];
		a_write[GIndex(i, height - 1, width)] = (fill_value == 2) ? -a_read[GIndex(i, height - 2, width)] : a_read[GIndex(i, height - 2, width)];
	}

	for (int i = 1; i < height - 1; ++i) {
		a_write[GIndex(0, i, width)] = (fill_value == 1) ? -a_read[GIndex(i, 1, width)] : a_read[GIndex(i, 1, width)];
		a_write[GIndex(width - 1, i, width)] = (fill_value == 1) ? -a_read[GIndex(width - 2, i, width)] : a_read[GIndex(width - 2, i, width)];
	}

	/*for (i = 1; i <= N; i++) {
		x[IX(0, i)] = b == 1 ? –x[IX(1, i)] : x[IX(1, i)];
		x[IX(N + 1, i)] = b == 1 ? –x[IX(N, i)] : x[IX(N, i)];
		x[IX(i, 0)] = b == 2 ? –x[IX(i, 1)] : x[IX(i, 1)];
		x[IX(i, N + 1)] = b == 2 ? –x[IX(i, N)] : x[IX(i, N)];
	}

	a_write[GIndex(0, 0, width)] = 0.5 * (a_read[GIndex(1, 0, width)] + a_read[GIndex(0, 1, width)]);
	a_write[GIndex(0, height - 1, width)] = 0.5 * (a_read[GIndex(1, height - 1, width)] + a_read[GIndex(0, height - 2, width)]);

	a_write[GIndex(width - 1, 0, width)] = 0.5 * (a_read[GIndex(width - 2, 0, width)] + a_read[GIndex(width - 1, 1, width)]);
	a_write[GIndex(width - 1, height - 1, width)] = 0.5 * (a_read[GIndex(width - 2, height - 1, width)] + a_read[GIndex(width - 1, height - 2, width)]);

	/*x[IX(0, 0)] = 0.5 * (x[IX(1, 0)] + x[IX(0, 1)]);
	x[IX(0, N + 1)] = 0.5 * (x[IX(1, N + 1)] + x[IX(0, N)]);
	x[IX(N + 1, 0)] = 0.5 * (x[IX(N, 0)] + x[IX(N + 1, 1)]);
	x[IX(N + 1, N + 1)] = 0.5 * (x[IX(N, N + 1)] + x[IX(N + 1, N)]);
}

void Smoke::fill_boundaries(float *a, float fill_value) {
	for (int i = 1; i < width - 1; ++i) {
		a[GIndex(i, 0, width)] = (fill_value == 2) ? -a[GIndex(i, 1, width)] : a[GIndex(i, 1, width)];
		a[GIndex(i, height - 1, width)] = (fill_value == 2) ? -a[GIndex(i, height - 2, width)] : a[GIndex(i, height - 2, width)];
	}

	for (int i = 1; i < height - 1; ++i) {
		a[GIndex(0, i, width)] = (fill_value == 1) ? -a[GIndex(i, 1, width)] : a[GIndex(i, 1, width)];
		a[GIndex(width - 1, i, width)] = (fill_value == 1) ? -a[GIndex(width - 2, i, width)] : a[GIndex(width - 2, i, width)];
	}

	a[GIndex(0, 0, width)] = 0.5 * (a[GIndex(1, 0, width)] + a[GIndex(0, 1, width)]);
	a[GIndex(0, height - 1, width)] = 0.5 * (a[GIndex(1, height - 1, width)] + a[GIndex(0, height - 2, width)]);

	a[GIndex(width - 1, 0, width)] = 0.5 * (a[GIndex(width - 2, 0, width)] + a[GIndex(width - 1, 1, width)]);
	a[GIndex(width - 1, height - 1, width)] = 0.5 * (a[GIndex(width - 2, height - 1, width)] + a[GIndex(width - 1, height - 2, width)]);
}*/

void Smoke::attach_density_source(int x, int y, int size, float strength) {
	SmokeSource s;
	s.x = x % N;
	s.y = y % N;
	s.size = size;
	s.strength = strength;
	d_sources->push_back(s);
}

void Smoke::attach_velocity_source(float x, float y, float dir_x, float dir_y, float strength) {
	VelSource s;
	s.x = x;
	s.y = y;
	s.dir_x = dir_x;
	s.dir_y = dir_y;
	s.strength = strength;
	v_sources->push_back(s);
}

void Smoke::set_diffusion_rate(float rate) {
	diffusion_rate = rate;
}

void Smoke::set_viscosity(float vis) {
	viscosity = vis;
}

void Smoke::add_density(int x, int y, float amount) {
	d[ix(x % (N - 1), y % (N - 1))] += amount;
}

void Smoke::add_velocity(int x, int y, float dir_x, float dir_y) {

	u[ix(x % (N - 1), y % (N - 1))] += dir_x;
	v[ix(x % (N - 1), y % (N - 1))] += dir_y;

}

// Uses Gauss-Seidel iterative method to solve linear system of equations, that can be represented as diagonally dominant matrix
void Smoke::_solve_linear(float *x, float *x0, float constant, float divident, int bnd_mode) {

	for (int k = 0; k < gauss_seidel_it_count; ++k) {

		for (int i = 1; i < N - 1; ++i) {

			for (int j = 1; j < N - 1; ++j) {

				x[ix(i, j)] = (x0[ix(i, j)] + constant * (x[ix(i - 1, j)] + x[ix(i + 1, j)] + x[ix(i, j - 1)] + x[ix(i, j + 1)])) * divident;
			}
		}
		_handle_boudaries(bnd_mode, x);
	}
}

void Smoke::_handle_boudaries(int bounding_mode, float *x) {

	// Borders
	for (int i = 1; i < N - 1; ++i) {

		x[ix(0, i)] = (bounding_mode == BND_VER_NEGATE) ? -x[ix(1, i)] : x[ix(1, i)];
		x[ix(N - 1, i)] = (bounding_mode == BND_VER_NEGATE) ? -x[ix(N - 2, i)] : x[ix(N - 2, i)];

		x[ix(i, 0)] = (bounding_mode == BND_HOR_NEGATE) ? -x[ix(i, 1)] : x[ix(i, 1)];
		x[ix(i, N - 1)] = (bounding_mode == BND_HOR_NEGATE) ? -x[ix(i, N - 2)] : x[ix(i, N - 2)];
	}

	// Corners
	x[ix(0, 0)] = 0.5 * (x[ix(1, 0)] + x[ix(0, 1)]);
	x[ix(0, N - 1)] = 0.5 * (x[ix(1, N - 1)] + x[ix(0, N - 2)]);
	x[ix(N - 1, 0)] = 0.5 * (x[ix(N - 2, 0)] + x[ix(N - 1, 1)]);
	x[ix(N - 1, N - 1)] = 0.5 * (x[ix(N - 2, N - 1)] + x[ix(N - 1, N - 2)]);
}

void Smoke::_apply_density_sources(float dt) {

	PoolVector<SmokeSource>::Read source = d_sources->read();

	for (int k = 0; k < d_sources->size(); ++k) {
		SmokeSource s = source[k];

		for (int i = s.x - s.size / 2; i < (s.x + s.size / 2); ++i) {
			for (int j = s.y - s.size / 2; j < (s.y + s.size / 2); ++j) {
				d0[ix(i, j)] += dt * s.strength;
			}
		}
	}
}

void Smoke::_apply_velocity_sources(float dt) {

	PoolVector<VelSource>::Read sources = v_sources->read();

	for (int k = 0; k < v_sources->size(); ++k) {
		VelSource s = sources[k];
		int i = static_cast<int>(s.x);
		int j = static_cast<int>(s.y);

		int ii = i + 1;
		int jj = j + 1;

		u0[ix(i, j)] += dt * s.dir_x * s.strength * (s.x - static_cast<float>(i));
		v0[ix(i, j)] += dt * s.dir_y * s.strength * (s.y - static_cast<float>(j));

		u0[ix(ii, j)] += dt * s.dir_x * s.strength * (1.0f - s.x + static_cast<float>(i));
		v0[ix(ii, j)] += dt * s.dir_y * s.strength * (s.y - static_cast<float>(j));

		u0[ix(i, jj)] += dt * s.dir_x * s.strength * (s.x - static_cast<float>(i));
		v0[ix(i, jj)] += dt * s.dir_y * s.strength * (1.0f - s.y + static_cast<float>(j));

		u0[ix(ii, jj)] += dt * s.dir_x * s.strength * (1- s.x + static_cast<float>(i));
		v0[ix(ii, jj)] += dt * s.dir_y * s.strength * (1.0f - s.y + static_cast<float>(j));
	}
}

void Smoke::_diffuse(float *x, float *x0, float rate, float dt, int bnd_mode) {
	float a = dt * rate * (N - 2) * (N - 2);

	_solve_linear(x, x0, a, 1 / (1 + 4 * a), bnd_mode);
}

void Smoke::_advect(float *x, float *x0, float *vel_x, float *vel_y, float dt, int bnd_mode) {

	float i0, i1, j0, j1;

	float dx = dt * static_cast<float>(N - 2);

	float s0, s1, t0, t1;
	float tmp1, tmp2, _x, _y;

	float N_f = static_cast<float>(N);
	float ifloat, jfloat;
	int i, j;

	for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++) {
		for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++) {
			tmp1 = dx * vel_x[ix(i, j)];
			tmp2 = dx * vel_y[ix(i, j)];
			_x = ifloat - tmp1;
			_y = jfloat - tmp2;

			if (_x < 0.5f)
				_x = 0.5f;
			if (_x > N_f - 0.5f)
				_x = N_f - 0.5f;
			i0 = floor(_x);
			i1 = i0 + 1.0f;
			if (_y < 0.5f)
				_y = 0.5f;
			if (_y > N_f - 0.5f)
				_y = N_f - 0.5f;
			j0 = floor(_y);
			j1 = j0 + 1.0f;

			s1 = _x - i0;
			s0 = 1.0f - s1;
			t1 = _y - j0;
			t0 = 1.0f - t1;

			int i0i = int(i0);
			int i1i = int(i1);
			int j0i = int(j0);
			int j1i = int(j1);

			x[ix(i, j)] =
					s0 * (t0 * x0[ix(i0i, j0i)] + t1 * x0[ix(i0i, j1i)]) +
					s1 * (t0 * x0[ix(i1i, j0i)] + t1 * x0[ix(i1i, j1i)]);
		}
	}

	_handle_boudaries(bnd_mode, x);
}

void Smoke::_project(float *vel_x, float *vel_y, float *p, float *div) {

	float h = 1.0f / N;
	for (int i = 1; i < N - 1; ++i) {

		for (int j = 1; j < N - 1; ++j) {

			div[ix(i, j)] = -0.5f * h * (vel_x[ix(i + 1, j)] - vel_x[ix(i - 1, j)] + vel_y[ix(i, j + 1)] - vel_y[ix(i, j - 1)]);
			p[ix(i, j)] = 0.0f;
		}
	}
	_handle_boudaries(BND_MISC, div);
	_handle_boudaries(BND_MISC, p);

	_solve_linear(p, div, 1.0f, 0.25, BND_MISC);

	float N_f = static_cast<float>(N);

	for (int i = 1; i < N - 1; ++i) {
		for (int j = 1; j < N - 1; ++j) {
			vel_x[ix(i, j)] -= 0.5 * (p[ix(i + 1, j)] - p[ix(i - 1, j)]) * N_f;
			vel_y[ix(i, j)] -= 0.5 * (p[ix(i, j + 1)] - p[ix(i, j - 1)]) * N_f;
		}
	}

	_handle_boudaries(BND_VER_NEGATE, vel_x);
	_handle_boudaries(BND_HOR_NEGATE, vel_y);
}

void Smoke::set_texture(const Ref<ImageTexture> &p_texture) {
	texture_ref = p_texture;
}

Ref<ImageTexture> Smoke::get_texture() const {
	return texture_ref;
}

void Smoke::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update", "dt"), &Smoke::update);
	ClassDB::bind_method(D_METHOD("attach_density_source", "x", "y", "size", "strength"), &Smoke::attach_density_source);
	ClassDB::bind_method(D_METHOD("attach_velocity_source", "x", "y", "dir_x", "dir_y", "strength"), &Smoke::attach_velocity_source);
	ClassDB::bind_method(D_METHOD("set_diffusion_rate", "rate"), &Smoke::set_diffusion_rate);
	ClassDB::bind_method(D_METHOD("set_viscosity", "vis"), &Smoke::set_viscosity);
	ClassDB::bind_method(D_METHOD("add_density", "x", "y", "amount"), &Smoke::add_density);
	ClassDB::bind_method(D_METHOD("add_velocity", "x", "y", "dir_x", "dir_y"), &Smoke::add_velocity);
	ClassDB::bind_method(D_METHOD("set_relaxation_iteration_count", "it_count"), &Smoke::set_relaxation_iteration_count);
	ClassDB::bind_method(D_METHOD("init", "size", "texture"), &Smoke::init);
	ClassDB::bind_method(D_METHOD("get_texture"), &Smoke::get_texture);
	ClassDB::bind_method(D_METHOD("set_texture"), &Smoke::set_texture);
}

#undef GIndex
#undef ix
