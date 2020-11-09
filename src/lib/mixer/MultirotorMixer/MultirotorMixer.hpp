/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <mixer/Mixer/Mixer.hpp>

/**
 * Supported multirotor geometries.
 *
 * Values are generated by the px_generate_mixers.py script and placed to mixer_multirotor_normalized.generated.h
 */
typedef uint8_t MultirotorGeometryUnderlyingType;
enum class MultirotorGeometry : MultirotorGeometryUnderlyingType;

/**
 * Multi-rotor mixer for pre-defined vehicle geometries.
 *
 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to
 * a set of outputs based on the configured geometry.
 */
class MultirotorMixer : public Mixer
{
public:
	/**

	 * Precalculated rotor mix.
	 */
	struct Rotor {
		float	roll_scale;		/**< scales roll for this rotor */
		float	pitch_scale;	/**< scales pitch for this rotor */
		float	yaw_scale;		/**< scales yaw for this rotor */
		float	thrust_scale;	/**< scales thrust for this rotor */
	};

	float motor_1_failure_gains[6] = {0,0.599,1.7998,1.49998,0.89999,1.1999};

	bool updated = false;
	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param geometry		The selected geometry.
	 * @param roll_scale		Scaling factor applied to roll inputs
	 *				compared to thrust.
	 * @param pitch_scale		Scaling factor applied to pitch inputs
	 *				compared to thrust.
	 * @param yaw_wcale		Scaling factor applied to yaw inputs compared
	 *				to thrust.
	 * @param idle_speed		Minimum rotor control output value; usually
	 *				tuned to ensure that rotors never stall at the
	 * 				low end of their control range.
	 */
	MultirotorMixer(ControlCallback control_cb, uintptr_t cb_handle, MultirotorGeometry geometry,
			float roll_scale, float pitch_scale, float yaw_scale, float idle_speed);

	/**
	 * Constructor (for testing).
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param rotors		control allocation matrix
	 * @param rotor_count		length of rotors array (= number of motors)
	 */
	MultirotorMixer(ControlCallback control_cb, uintptr_t cb_handle,Rotor *rotors, unsigned rotor_count);
	virtual ~MultirotorMixer();

	// no copy, assignment, move, move assignment
	MultirotorMixer(const MultirotorMixer &) = delete;
	MultirotorMixer &operator=(const MultirotorMixer &) = delete;
	MultirotorMixer(MultirotorMixer &&) = delete;
	MultirotorMixer &operator=(MultirotorMixer &&) = delete;

	/**
	 * Factory method.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new MultirotorMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static MultirotorMixer *from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
					  unsigned &buflen);

	unsigned		mix(float *outputs, unsigned space) override;

	uint16_t		get_saturation_status() override { return _saturation_status.value; }

	void			groups_required(uint32_t &groups) override { groups |= (1 << 0); }

	/**
	 * @brief      Update slew rate parameter. This tells the multicopter mixer
	 *             the maximum allowed change of the output values per cycle.
	 *             The value is only valid for one cycle, in order to have continuous
	 *             slew rate limiting this function needs to be called before every call
	 *             to mix().
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	void 			set_max_delta_out_once(float delta_out_max) override { _delta_out_max = delta_out_max; }

	unsigned		set_trim(float trim) override { return _rotor_count; }
	unsigned		get_trim(float *trim) override { return _rotor_count; }

	void 	    motor_failure_1() override {
			if (updated) return;
			updated = true;
			printf("updating rotor status\n");
			return;
			/*float values[6][4] = {
					{ -0.000000,  0.000000, -0.000000,  0.000000 },
					{  0.319471, -0.000000,  0.099835,  0.332815 },
					{  0.239977,  0.969854,  0.599941,  1.000000 },
					{ -1.000000, -0.808291,  1.000000,  0.833415 },
					{ -0.120000,  0.484976, -0.300001,  0.500050 },
					{  0.799944, -0.646587, -0.799944,  0.666685 },
				};
								{ -0.659749,  0.000000, -0.250000,  0.666587 },
				{  0.000000, -0.000000,  0.000000,  0.000000 },
				{  0.000000,  0.000000,  0.000000,  0.000000 },
				{ -0.412393, -0.333333,  0.500060,  0.333333 },
				{ -0.247436,  1.000000, -0.750090,  1.000000 },
				{  0.824687, -0.666587, -1.000000,  0.666587 },
				""""""


			float values2[6][6]{
				{  0.000000,  0.000000,  0.000000,  0.000000 },
				{  0.866025, -0.866025,  0.000000, -0.000000 },
				{  0.000000,  0.866025, -0.000000,  1.500000 },
				{ -0.866025, -0.000000, -0.000000,  1.500000 }
			};

			for (unsigned i = 0; i < _rotor_count; i++) {
				_rotors[i].roll_scale = values2[i][0];
				_rotors[i].pitch_scale = values2[i][1];
				_rotors[i].yaw_scale = values2[i][2];
				_rotors[i].thrust_scale = values2[i][3];
			}
			return;
			*/
	}


	/**
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to motor control signal output.
	 *
	 * @param[in]  val   The value
	 */
	void			set_thrust_factor(float val) override { _thrust_factor = math::constrain(val, 0.0f, 1.0f); }

	void 			set_airmode(Airmode airmode) override { _airmode = airmode; }

	unsigned		get_multirotor_count() override { return _rotor_count; }

	union saturation_status {
		struct {
			uint16_t valid		: 1; // 0 - true when the saturation status is used
			uint16_t motor_pos	: 1; // 1 - true when any motor has saturated in the positive direction
			uint16_t motor_neg	: 1; // 2 - true when any motor has saturated in the negative direction
			uint16_t roll_pos	: 1; // 3 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 4 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 5 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 6 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 7 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 8 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_pos	: 1; // 9 - true when a positive thrust demand change will increase saturation
			uint16_t thrust_neg	: 1; //10 - true when a negative thrust demand change will increase saturation
		} flags;
		uint16_t value;
	};

private:
	/**
	 * Computes the gain k by which desaturation_vector has to be multiplied
	 * in order to unsaturate the output that has the greatest saturation.
	 * @see also minimize_saturation().
	 *
	 * @return desaturation gain
	 */
	float compute_desaturation_gain(const float *desaturation_vector, const float *outputs, saturation_status &sat_status,
					float min_output, float max_output) const;

	/**
	 * Minimize the saturation of the actuators by adding or substracting a fraction of desaturation_vector.
	 * desaturation_vector is the vector that added to the output outputs, modifies the thrust or angular
	 * acceleration on a specific axis.
	 * For example, if desaturation_vector is given to slide along the vertical thrust axis (thrust_scale), the
	 * saturation will be minimized by shifting the vertical thrust setpoint, without changing the
	 * roll/pitch/yaw accelerations.
	 *
	 * Note that as we only slide along the given axis, in extreme cases outputs can still contain values
	 * outside of [min_output, max_output].
	 *
	 * @param desaturation_vector vector that is added to the outputs, e.g. thrust_scale
	 * @param outputs output vector that is modified
	 * @param sat_status saturation status output
	 * @param min_output minimum desired value in outputs
	 * @param max_output maximum desired value in outputs
	 * @param reduce_only if true, only allow to reduce (substract) a fraction of desaturation_vector
	 */
	void minimize_saturation(const float *desaturation_vector, float *outputs, saturation_status &sat_status,
				 float min_output = 0.f, float max_output = 1.f, bool reduce_only = false) const;

	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: airmode for roll/pitch:
	 * thrust is increased/decreased as much as required to meet the demanded roll/pitch.
	 * Yaw is not allowed to increase the thrust, @see mix_yaw() for the exact behavior.
	 */
	inline void mix_airmode_rp(float roll, float pitch, float yaw, float thrust, float *outputs);

	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: full airmode for roll/pitch/yaw:
	 * thrust is increased/decreased as much as required to meet demanded the roll/pitch/yaw,
	 * while giving priority to roll and pitch over yaw.
	 */
	inline void mix_airmode_rpy(float roll, float pitch, float yaw, float thrust, float *outputs);


	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: no airmode, thrust is NEVER increased to meet the demanded
	 * roll/pitch/yaw. Instead roll/pitch/yaw is reduced as much as needed.
	 * Thrust can be reduced to unsaturate the upper side.
	 * @see mix_yaw() for the exact yaw behavior.
	 */
	inline void mix_airmode_disabled(float roll, float pitch, float yaw, float thrust, float *outputs);

	/**
	 * Mix yaw by updating an existing output vector (that already contains roll/pitch/thrust).
	 *
	 * Desaturation behavior: thrust is allowed to be decreased up to 15% in order to allow
	 * some yaw control on the upper end. On the lower end thrust will never be increased,
	 * but yaw is decreased as much as required.
	 *
	 * @param yaw demanded yaw
	 * @param outputs output vector that is updated
	 */
	inline void mix_yaw(float yaw, float *outputs);

	void update_saturation_status(unsigned index, bool clipping_high, bool clipping_low_roll_pitch, bool clipping_low_yaw);

	float				_roll_scale{1.0f};
	float				_pitch_scale{1.0f};
	float				_yaw_scale{1.0f};
	float				_idle_speed{0.0f};
	float 				_delta_out_max{0.0f};
	float 				_thrust_factor{0.0f};

	Airmode				_airmode{Airmode::disabled};

	saturation_status		_saturation_status{};

	unsigned			_rotor_count;
	Rotor			*_rotors;

	float 				*_outputs_prev{nullptr};
	float 				*_tmp_array{nullptr};
};
