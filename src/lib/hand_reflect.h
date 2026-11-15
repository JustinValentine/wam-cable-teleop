#pragma once

/* Wrist load scaling for unequal hand points.
 *
 * The wrists are coupled joint to joint. The follower's grasp point is `grasp`
 * metres out from its wrist centre along the tool axis, the axis of its J7,
 * and the leader's handle only `hand` metres out from its own. A force at the
 * grasp point loads the follower's J6 over the whole lever, its J5 over the
 * lever times the sine of J6, and its J7 not at all; the handle sits the same
 * way on the leader's J7 and J6. So a force on the gripper should reach each
 * of those two as scale = hand / grasp of its partner's torque, and J8, whose
 * partner is the follower's J7, is left alone. The position coupling delivers
 * something else: whatever the follower's position controller pushes with,
 * which on a WAM wrist is about half the load at rest (the wrist's friction
 * holds the rest).
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
 * `grasp` is the lever as the follower's sensor sees it
 * (scripts/grasp_lever.py). A moment about the grasp point is scaled with the
 * rest: telling it from the force's share needs the sensor's measuring origin
 * along the tool axis, which is not calibrated, and an error there would turn
 * part of a carried weight into a moment passed in full.
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
