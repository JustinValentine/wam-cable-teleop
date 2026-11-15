#pragma once

/* Wrist load scaling for unequal hand points.
 *
 * The wrists are coupled joint to joint. The follower's grasp point is `grasp`
 * metres from its wrist centre and the leader's handle only `hand` metres from
 * its own, so a force on the gripper should reach the operator's wrist as
 * scale = hand / grasp of the follower's wrist torque. The position coupling
 * delivers something else: whatever the follower's position controller pushes
 * with, which on a WAM wrist is about half the load at rest (the wrist's
 * friction holds the rest), on a lever three times too short.
 *
 * So at low frequency the leader's wrist torque is steered to the measured
 * torque, scaled:
 *
 *     add = gain * lowpass( scale * sensed - delivered )
 *
 * sensed: the follower's measured wrist torque, mapped to this joint.
 * delivered: what this wrist's position controller and wrist force channel
 * apply. With gain 1 the wrist settles on scale * sensed. Above the filter's
 * corner the coupling acts as before, so a contact starts as stiff as it did.
 *
 * No libbarrett: tested offline (test/test_hand_reflect.cpp). */

#include <cmath>

namespace hand_reflect {

struct Correction {
	double filtered[3];

	Correction() { reset(); }

	void reset() {
		filtered[0] = filtered[1] = filtered[2] = 0.0;
	}

	// One cycle. out = gain * filtered, at most `limit` Nm per joint.
	void step(const double* want, const double* delivered, double gain, double cutoffHz, double period, double limit,
	          double* out) {
		const double k = cutoffHz > 0.0 ? 1.0 - std::exp(-2.0 * 3.14159265358979323846 * cutoffHz * period) : 1.0;
		for (int i = 0; i < 3; ++i) {
			filtered[i] += k * ((want[i] - delivered[i]) - filtered[i]);
			double v = gain * filtered[i];
			if (v > limit) v = limit;
			if (v < -limit) v = -limit;
			out[i] = v;
		}
	}
};

}  // namespace hand_reflect
