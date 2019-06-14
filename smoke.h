/*****************************************
*	Implementation by M. Gornicki
*	Hugely based on work of Jos Stam
*	and explanation by Mike Ash 
******************************************/
#ifndef SMOKE_H
#define SMOKE_H

#include "scene/resources/texture.h"

class Smoke : public Reference {
	GDCLASS(Smoke, Reference);

	struct SmokeSource {
		int x, y;
		int size, strength;
	};

	struct VelSource {
		float x, y;
		float strength;
		float dir_x, dir_y;
	};

	// Densities
	float_t *d;
	float_t *d0;

	// Velocities
	float_t *u;
	float_t *u0;
	float_t *v;
	float_t *v0;

	PoolVector<SmokeSource> *d_sources;
	PoolVector<VelSource> *v_sources;
	PoolVector<uint8_t> *buffer1;
	Ref<Image> image = memnew(Image);

	int N;
	size_t gauss_seidel_it_count;
	float diffusion_rate;
	float viscosity;
	Ref<ImageTexture> texture_ref;

	static const int BND_MISC = 0;
	static const int BND_VER_NEGATE = 1;
	static const int BND_HOR_NEGATE = 2;

protected:
	static void _bind_methods();

public:
	Smoke();
	~Smoke();
	void init(int size, const Ref<ImageTexture> &p_texture);
	void set_texture(const Ref<ImageTexture> &p_texture);
	Ref<ImageTexture> get_texture() const;
	void update(float dt);
	void set_relaxation_iteration_count(int it_count);
	void attach_density_source(int x, int y, int size, float strength);
	void attach_velocity_source(float x, float y, float dir_x, float dir_y, float strength);
	void set_diffusion_rate(float rate);
	void set_viscosity(float vis);
	void add_density(int x, int y, float amount);
	void add_velocity(int x, int y, float dir_x, float dir_y);

private:
	// Helpers
	void _solve_linear(float *x, float *x0, float constant, float divident, int bnd_mode);
	void _handle_boudaries(int bounding_mode, float *x);
	void _apply_density_sources(float dt);
	void _apply_velocity_sources(float dt);

	// Main steps
	void _diffuse(float *x, float *x0, float rate, float dt, int bnd_mode);
	void _advect(float *x, float *x0, float *vel_x, float *vel_y, float dt, int bnd_mode);
	void _project(float *vel_x, float *vel_y, float *p, float *div);
};

#endif // SMOKE_H
