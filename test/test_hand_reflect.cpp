/* Offline checks of the wrist load scaling (lib/hand_reflect.h): its statics
 * on loads like the recorded ones, and its filter.
 * Run: devel/lib/wam_teleop/hand_reflect_tests */

#include <cmath>
#include <cstdio>

#include "hand_reflect.h"

namespace {

int failures = 0;
int checks = 0;

void check(bool ok, const char* what) {
	++checks;
	if (!ok) {
		++failures;
		printf("FAIL: %s\n", what);
	}
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

const double SCALE = 0.07 / 0.21;

// Settles a correction and returns what it adds.
void settle(const double* want, const double* delivered, double gain, double* out) {
	hand_reflect::Correction c;
	for (int i = 0; i < 5000; ++i) {
		c.step(want, delivered, gain, 1.0, 0.002, 3.0, out);
	}
}

void testStatics() {
	// A 3 lb weight in a level gripper: 2.8 Nm on the follower's wrist, of
	// which the coupling delivers 1.7 Nm (its friction holds the rest).
	const double sensed[3] = {0.0, 2.8, 0.0};
	const double heavy[3] = {0.0, 1.7, 0.0};
	const double want[3] = {SCALE * sensed[0], SCALE * sensed[1], SCALE * sensed[2]};
	double out[3];
	settle(want, heavy, 1.0, out);
	check(near(heavy[1] + out[1], 2.8 * SCALE, 1e-6), "a heavy load settles on the measured torque times hand / grasp");
	check(out[1] < 0.0 && std::fabs(out[1]) < heavy[1], "... by taking torque away, less than the coupling gives");

	// A light load that the follower's friction hides from the coupling.
	const double sensedLight[3] = {0.0, 0.4, 0.0};
	const double light[3] = {0.0, 0.04, 0.0};
	const double wantLight[3] = {0.0, SCALE * 0.4, 0.0};
	settle(wantLight, light, 1.0, out);
	check(near(light[1] + out[1], 0.4 * SCALE, 1e-6) && out[1] > 0.0, "a light load the coupling hides is added");

	// Nothing measured: the coupling's own torque (the follower's drag) is taken out.
	const double none[3] = {0.0, 0.0, 0.0};
	const double drag[3] = {0.3, -0.2, 0.0};
	settle(none, drag, 1.0, out);
	check(near(drag[0] + out[0], 0.0, 1e-6) && near(drag[1] + out[1], 0.0, 1e-6), "with nothing measured the coupling's steady torque is cancelled");

	// Half the gain: half way between the coupling and the scaled measurement.
	settle(want, heavy, 0.5, out);
	check(near(heavy[1] + out[1], 0.5 * heavy[1] + 0.5 * 2.8 * SCALE, 1e-6), "gain 0.5: half way");
	(void) sensedLight;
}

void testFilter() {
	using hand_reflect::Correction;
	const double want[3] = {0.5, -1.0, 0.0};
	const double delivered[3] = {1.5, -3.0, 0.0};
	double out[3];

	Correction off;
	for (int i = 0; i < 2000; ++i) off.step(want, delivered, 0.0, 1.0, 0.002, 3.0, out);
	check(out[0] == 0.0 && out[1] == 0.0 && out[2] == 0.0, "gain 0: no torque");

	Correction c;
	c.step(want, delivered, 1.0, 1.0, 0.002, 3.0, out);
	check(std::fabs(out[1]) < 0.03, "the first cycle adds almost nothing (1 Hz low-pass)");
	for (int i = 1; i < 80; ++i) c.step(want, delivered, 1.0, 1.0, 0.002, 3.0, out);  // 0.16 s
	check(out[1] > 0.55 * 2.0 && out[1] < 0.70 * 2.0, "63 % after one time constant, 0.16 s");
	for (int i = 0; i < 3000; ++i) c.step(want, delivered, 1.0, 1.0, 0.002, 3.0, out);
	check(near(out[0], -1.0, 1e-6) && near(out[1], 2.0, 1e-6), "it settles on want - delivered");

	const double big[3] = {0.0, 20.0, 0.0}, none[3] = {0.0, 0.0, 0.0};
	Correction limited;
	for (int i = 0; i < 3000; ++i) limited.step(big, none, 1.0, 1.0, 0.002, 3.0, out);
	check(out[1] == 3.0, "never more than the limit");
	limited.reset();
	limited.step(none, none, 1.0, 1.0, 0.002, 3.0, out);
	check(out[1] == 0.0, "reset empties the filter");
}

}  // namespace

int main() {
	testStatics();
	testFilter();
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
