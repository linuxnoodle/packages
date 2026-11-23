// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the lens model and the IPD reply parser, with synthetic values
 * (no device data): file parsing, the polynomial, chromatic aberration, the
 * fitted radius, the field of view, the only MCU request the driver sends and
 * the port lock it shares with pico4-fan.
 */

#include "pico4_ipd.h"
#include "pico4_lens.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

#define NEAR(a, b) (fabs((a) - (b)) < 1e-9)

// Same structure as the factory file: numbers as strings, centre offsets in mm.
static const char *optical =
    "{\"distortion_parameter\": {"
    "\"left\": {\"F\": \"0.025\", \"A0\": \"1\", \"A1\": \"0.5\", \"A2\": \"0\", \"A3\": \"0\", \"A4\": \"0\","
    " \"A5\": \"0\", \"A6\": \"0\", \"A7\": \"0\", \"A8\": \"0\", \"A9\": \"0\", \"A10\": \"0\"},"
    "\"right\": {\"F\": \"0.025\", \"A0\": \"1\", \"A1\": \"0\", \"A2\": \"0\", \"A3\": \"0\", \"A4\": \"0\","
    " \"A5\": \"0\", \"A6\": \"0\", \"A7\": \"0\", \"A8\": \"0\", \"A9\": \"0\", \"A10\": \"0\"}},"
    "\"lens_center_gos\": {\"centerx\": {\"left\": \"0.5\", \"right\": \"0\"},"
    " \"centery\": {\"left\": \"0\", \"right\": \"-0.25\"}}}";

// Same structure as the stock lens configuration.
static const char *config =
    "{\"configData\": ["
    "{\"keyName\": \"sdk_lens_chromaticAberration0\", \"keyValue\": \"0.01\"},"
    "{\"keyName\": \"sdk_lens_chromaticAberration1\", \"keyValue\": \"0\"},"
    "{\"keyName\": \"sdk_lens_chromaticAberration2\", \"keyValue\": \"-0.02\"},"
    "{\"keyName\": \"sdk_lens_chromaticAberration3\", \"keyValue\": \"0\"},"
    "{\"keyName\": \"sdk_lens_chromaticAberration4\", \"keyValue\": \"0\"},"
    "{\"keyName\": \"sdk_lens_chromaticAberration5\", \"keyValue\": \"0\"},"
    "{\"keyName\": \"sdk_lens_distortionCoefficients1\", \"keyValue\": \"9\"}]}";

// A factory file that names its eyes' lenses (each eye a batch of its own) and its lens centre offsets, but
// no polynomial: such files exist, and PICO OS runs each lens's batch's profile for them.
static const char *optical_without_polynomial =
    "{\"lens_data\": {"
    "\"left\": {\"lens_model\": \"0B\", \"lens_hole\": \"3\", \"lens_date\": \"20220815\"},"
    "\"right\": {\"lens_model\": \"0T\", \"lens_hole\": \"4\", \"lens_date\": \"20220815\"}},"
    "\"lens_center_gos\": {\"centerx\": {\"left\": \"0.5\", \"right\": \"0\"},"
    " \"centery\": {\"left\": \"0\", \"right\": \"-0.25\"}}}";

// The same with the left eye's polynomial only.
static const char *optical_left_polynomial =
    "{\"distortion_parameter\": {"
    "\"left\": {\"F\": \"0.025\", \"A0\": \"1\", \"A1\": \"0.5\", \"A2\": \"0\", \"A3\": \"0\", \"A4\": \"0\","
    " \"A5\": \"0\", \"A6\": \"0\", \"A7\": \"0\", \"A8\": \"0\", \"A9\": \"0\", \"A10\": \"0\"}},"
    "\"lens_data\": {"
    "\"left\": {\"lens_model\": \"0B\", \"lens_hole\": \"3\", \"lens_date\": \"20220815\"},"
    "\"right\": {\"lens_model\": \"0T\", \"lens_hole\": \"4\", \"lens_date\": \"20220815\"}},"
    "\"lens_center_gos\": {\"centerx\": {\"left\": \"0.5\", \"right\": \"0\"},"
    " \"centery\": {\"left\": \"0\", \"right\": \"-0.25\"}}}";

// Same structure as the table of lens batches: per entry a lens serial number or "default" and the profile
// "F,A0,...,A10" it links to. Not in sorted order, as the stock file is not; two entries are no lens batch's
// profile, one with another linkage and one with another key.
#define PROFILE(f, a0, a1) f "," a0 "," a1 ",0,0,0,0,0,0,0,0,0"
#define BATCH(sn, profile)                                                                                                 \
	"{\"_description\": \"Lens And Distortion Match Inf\", \"key\": \"lens_sn\", \"value\": \"" sn "\", \
     \"linkageKey\": \"sdk_distortion_para\", \"linkageValue\": \"" profile "\", \"rule\": 1, \"status\": 1},"
static const char *match_table =
    "{\"versionCode\": 3, \"status\": 1, "
    "\"path\": \"/system/etc/pxrconfig/rule/SDK_Lens_Distortion_MatchConfigFile.txt\", \"ruleData\": ["
    BATCH("default", PROFILE("0.027", "1", "0.4"))
    BATCH("m0B_h3_d20220901", PROFILE("0.027", "1", "0.6"))
    BATCH("m0T_h4_d20220815", PROFILE("0.026", "1", "0.45"))
    BATCH("m0B_h1_d20220701", PROFILE("0.027", "1", "0.41"))
    BATCH("m0B_h3_d20220701", PROFILE("0.027", "1", "0.42"))
    BATCH("m0B_h3_d20220801", PROFILE("0.0265", "1.0000005", "0.5"))
    "{\"key\": \"lens_sn\", \"value\": \"m0B_h3_d20220810\", \"linkageKey\": \"sdk_display_panel\", "
    "\"linkageValue\": \"" PROFILE("0.09", "9", "9") "\", \"rule\": 1, \"status\": 1},"
    "{\"key\": \"panel_sn\", \"value\": \"m0B_h3_d20220812\", \"linkageKey\": \"sdk_distortion_para\", "
    "\"linkageValue\": \"" PROFILE("0.08", "8", "8") "\", \"rule\": 1, \"status\": 1},"
    "{\"_description\": \"Lens And Distortion Match Inf\", \"key\": \"lens_sn\", \"value\": \"m0B_h3_d20221001\", "
    "\"linkageKey\": \"sdk_distortion_para\", \"linkageValue\": \"" PROFILE("0.027", "1", "0.7") "\", \"rule\": 1, "
    "\"status\": 1}]}";
#undef BATCH
#undef PROFILE

// Same structure as the stock lens configuration: one generic profile, numbers as strings.
#define SDK_COEFFICIENT(i, v) "{\"keyName\": \"sdk_lens_distortionCoefficients" #i "\", \"keyValue\": \"" v "\"},"
static const char *sdk_table =
    "{\"versionCode\": 5, \"configData\": ["
    "{\"keyName\": \"sdk_lens_metersPerTanAngleAtCenter\", \"keyValue\": \"0.027\"},"
    "{\"keyName\": \"sdk_lens_distortionEqnType\", \"keyValue\": \"2\"},"
    SDK_COEFFICIENT(0, "1")
    SDK_COEFFICIENT(1, "0.5")
    SDK_COEFFICIENT(2, "0") SDK_COEFFICIENT(3, "0") SDK_COEFFICIENT(4, "0")
    SDK_COEFFICIENT(5, "0") SDK_COEFFICIENT(6, "0") SDK_COEFFICIENT(7, "0")
    SDK_COEFFICIENT(8, "0") SDK_COEFFICIENT(9, "0") SDK_COEFFICIENT(10, "0")
    "{\"keyName\": \"sdk_lens_chromaticAberration0\", \"keyValue\": \"0\"}]}";

// Equation type 3 would not be the power series in r² the model reads.
static const char *sdk_other_eqn =
    "{\"configData\": ["
    "{\"keyName\": \"sdk_lens_metersPerTanAngleAtCenter\", \"keyValue\": \"0.027\"},"
    "{\"keyName\": \"sdk_lens_distortionEqnType\", \"keyValue\": \"3\"},"
    SDK_COEFFICIENT(0, "1") SDK_COEFFICIENT(1, "0") SDK_COEFFICIENT(2, "0")
    SDK_COEFFICIENT(3, "0") SDK_COEFFICIENT(4, "0") SDK_COEFFICIENT(5, "0")
    SDK_COEFFICIENT(6, "0") SDK_COEFFICIENT(7, "0") SDK_COEFFICIENT(8, "0")
    SDK_COEFFICIENT(9, "0") SDK_COEFFICIENT(10, "0")
    "{\"keyName\": \"sdk_lens_distortionMode\", \"keyValue\": \"1\"}]}";
#undef SDK_COEFFICIENT

int
main(void)
{
	struct pico4_lens lens[2];
	struct pico4_lens_ca ca;

	EXPECT(pico4_lens_parse_optical(optical, lens));
	EXPECT(NEAR(lens[0].f, 0.025) && NEAR(lens[0].k[1], 0.5) && NEAR(lens[1].k[0], 1));
	// The stock compositor's directions in panel coordinates (x right, y down): -centerx, -centery.
	EXPECT(NEAR(lens[0].cx, -0.0005) && NEAR(lens[1].cy, 0.00025));
	EXPECT(!pico4_lens_parse_optical("{\"distortion_parameter\": {\"left\": {\"F\": \"0.02\"}}}", lens) &&
	       pico4_lens_parse_optical(optical, lens));
	EXPECT(pico4_lens_parse_ca(config, &ca));
	EXPECT(NEAR(ca.ca[0], 0.01) && NEAR(ca.ca[2], -0.02));
	EXPECT(!pico4_lens_parse_ca("{\"configData\": []}", &ca) && pico4_lens_parse_ca(config, &ca));

	// A factory file without a polynomial: its lenses' serial numbers and its offsets still parse, per eye.
	char serial[PICO4_LENS_SERIAL_MAX] = "kept";
	EXPECT(!pico4_lens_parse_optical(optical_without_polynomial, lens));
	EXPECT(pico4_lens_parse_serial(optical_without_polynomial, 0, serial) &&
	       strcmp(serial, "m0B_h3_d20220815") == 0);
	EXPECT(pico4_lens_parse_serial(optical_without_polynomial, 1, serial) &&
	       strcmp(serial, "m0T_h4_d20220815") == 0);
	EXPECT(pico4_lens_parse_centers(optical_without_polynomial, lens));
	EXPECT(NEAR(lens[0].cx, -0.0005) && NEAR(lens[1].cy, 0.00025));
	EXPECT(!pico4_lens_parse_centers("{\"single_screen_centor\": {}}", lens));
	// The parts are strings in the stock file; plain numbers read the same. No lens for the eye, or no
	// such eye, is none, and tells of no serial number.
	EXPECT(pico4_lens_parse_serial(
	           "{\"lens_data\": {\"left\": {\"lens_model\": \"0B\", \"lens_hole\": 3, \"lens_date\": 20220815}}}", 0,
	           serial) &&
	       strcmp(serial, "m0B_h3_d20220815") == 0);
	EXPECT(!pico4_lens_parse_serial(optical, 0, serial) && serial[0] == '\0');
	EXPECT(!pico4_lens_parse_serial(optical_without_polynomial, 2, serial) && serial[0] == '\0');

	// Each eye's polynomial on its own; an incomplete one is none and leaves the lens as it was.
	EXPECT(pico4_lens_parse_factory(optical, 0, &lens[0]) && NEAR(lens[0].f, 0.025) && NEAR(lens[0].k[1], 0.5));
	EXPECT(pico4_lens_parse_factory(optical, 1, &lens[1]) && NEAR(lens[1].k[0], 1) && NEAR(lens[1].k[1], 0));
	EXPECT(!pico4_lens_parse_factory(optical_without_polynomial, 0, &lens[0]) && NEAR(lens[0].f, 0.025));
	EXPECT(!pico4_lens_parse_factory("{\"distortion_parameter\": {\"left\": {\"F\": \"0.02\"}}}", 0, &lens[0]) &&
	       NEAR(lens[0].f, 0.025));
	EXPECT(!pico4_lens_parse_factory(optical, 2, &lens[0]) && NEAR(lens[0].f, 0.025));

	// PICO OS's pick from the table of lens batches: the newest batch of the lens's mould and cavity
	// dated on or before the lens, its F and coefficients together; its own batch is at or below;
	// another cavity, a newer batch and the entries of another linkage and of another key play no part.
	EXPECT(pico4_lens_parse_match_table(match_table, "m0B_h3_d20220815", &lens[0]));
	EXPECT(NEAR(lens[0].f, 0.0265) && NEAR(lens[0].k[0], 1.0000005) && NEAR(lens[0].k[1], 0.5));
	EXPECT(NEAR(lens[0].cx, -0.0005));
	EXPECT(pico4_lens_parse_match_table(match_table, "m0T_h4_d20220815", &lens[0]) && NEAR(lens[0].f, 0.026) &&
	       NEAR(lens[0].k[1], 0.45));
	EXPECT(pico4_lens_parse_match_table(match_table, "m0B_h1_d20220815", &lens[0]) && NEAR(lens[0].f, 0.027) &&
	       NEAR(lens[0].k[1], 0.41));
	// A mould or cavity the table does not list, and a lens older than every batch of its own: "default".
	EXPECT(pico4_lens_parse_match_table(match_table, "m0Z_h1_d20220815", &lens[0]) && NEAR(lens[0].f, 0.027) &&
	       NEAR(lens[0].k[1], 0.4));
	EXPECT(pico4_lens_parse_match_table(match_table, "m0B_h3_d20220601", &lens[0]) && NEAR(lens[0].f, 0.027) &&
	       NEAR(lens[0].k[1], 0.4));
	// An entry whose values are not F and eleven coefficients (here eleven, then thirteen) is no profile,
	// and leaves the lens as it was; so is a serial number without a mould and cavity.
	EXPECT(!pico4_lens_parse_match_table(
	           "{\"ruleData\": [{\"key\": \"lens_sn\", \"value\": \"default\", \"linkageKey\": \"sdk_distortion_para\", "
	           "\"linkageValue\": \"0.027,1,0.4,0,0,0,0,0,0,0\"}]}",
	           "m0B_h3_d20220815", &lens[0]) &&
	       NEAR(lens[0].f, 0.027));
	EXPECT(!pico4_lens_parse_match_table(
	           "{\"ruleData\": [{\"key\": \"lens_sn\", \"value\": \"default\", \"linkageKey\": \"sdk_distortion_para\", "
	           "\"linkageValue\": \"0.027,1,0.4,0,0,0,0,0,0,0,0,0,0\"}]}",
	           "m0B_h3_d20220815", &lens[0]) &&
	       NEAR(lens[0].f, 0.027));
	EXPECT(!pico4_lens_parse_match_table("{\"ruleData\": []}", "m0B_h3_d20220815", &lens[0]));
	EXPECT(!pico4_lens_parse_match_table(match_table, "m0B_h3", &lens[0]));
	EXPECT(!pico4_lens_parse_match_table(match_table, "m0B", &lens[0]));

	// The stock lens configuration's generic profile: no offsets of its own, another equation type
	// refused, and the lens centre kept as it was.
	EXPECT(pico4_lens_parse_sdk_table(sdk_table, &lens[0]) && NEAR(lens[0].f, 0.027) && NEAR(lens[0].k[1], 0.5));
	EXPECT(NEAR(lens[0].cx, -0.0005));
	EXPECT(!pico4_lens_parse_sdk_table(sdk_other_eqn, &lens[0]) && NEAR(lens[0].f, 0.027));
	EXPECT(!pico4_lens_parse_sdk_table(config, &lens[0]) && NEAR(lens[0].f, 0.027));

	// Per eye, as PICO OS picks: the factory file's polynomial; else the lens's batch, whose serial
	// number is told; else the generic profile: without a table, without one for the lens, without
	// a factory file at all. The factory file's offsets go on both eyes' profiles.
	struct pico4_lens_loaded loaded;
	EXPECT(pico4_lens_load(optical, match_table, sdk_table, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_FACTORY && loaded.source[1] == PICO4_LENS_FACTORY);
	EXPECT(NEAR(lens[0].f, 0.025) && NEAR(lens[1].f, 0.025) && loaded.serial[0][0] == '\0' &&
	       loaded.serial[1][0] == '\0');
	EXPECT(NEAR(lens[0].cx, -0.0005) && NEAR(lens[1].cy, 0.00025));
	EXPECT(pico4_lens_load(optical_without_polynomial, match_table, sdk_table, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_BATCH && loaded.source[1] == PICO4_LENS_BATCH);
	EXPECT(NEAR(lens[0].f, 0.0265) && NEAR(lens[0].k[0], 1.0000005) && NEAR(lens[1].f, 0.026) &&
	       NEAR(lens[1].k[1], 0.45));
	EXPECT(strcmp(loaded.serial[0], "m0B_h3_d20220815") == 0 && strcmp(loaded.serial[1], "m0T_h4_d20220815") == 0);
	EXPECT(NEAR(lens[0].cx, -0.0005) && NEAR(lens[1].cy, 0.00025));
	// One eye's polynomial and not the other's: each eye on its own.
	EXPECT(pico4_lens_load(optical_left_polynomial, match_table, sdk_table, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_FACTORY && NEAR(lens[0].f, 0.025) && NEAR(lens[0].k[1], 0.5));
	EXPECT(loaded.source[1] == PICO4_LENS_BATCH && NEAR(lens[1].f, 0.026));
	// Without a table of lens batches, or without one for the lens: the generic profile, the lens's
	// serial number still told. Without a factory file: the generic profile and no serial number.
	EXPECT(pico4_lens_load(optical_without_polynomial, NULL, sdk_table, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_GENERIC && NEAR(lens[0].f, 0.027) && NEAR(lens[0].k[1], 0.5));
	EXPECT(strcmp(loaded.serial[0], "m0B_h3_d20220815") == 0);
	EXPECT(pico4_lens_load(optical_without_polynomial, "{\"ruleData\": []}", sdk_table, lens, &loaded));
	EXPECT(loaded.source[1] == PICO4_LENS_GENERIC && NEAR(lens[1].f, 0.027));
	EXPECT(pico4_lens_load(NULL, match_table, sdk_table, lens, &loaded));
	EXPECT(loaded.source[1] == PICO4_LENS_GENERIC && loaded.serial[1][0] == '\0');
	// Nothing at all: no model, none told of. One eye without a profile: no model for the headset, so
	// no offsets on the other eye's either.
	EXPECT(!pico4_lens_load(NULL, NULL, NULL, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_NONE && loaded.source[1] == PICO4_LENS_NONE);
	EXPECT(NEAR(lens[0].f, 0) && NEAR(lens[1].f, 0));
	EXPECT(!pico4_lens_load(optical_left_polynomial, NULL, NULL, lens, &loaded));
	EXPECT(loaded.source[0] == PICO4_LENS_FACTORY && loaded.source[1] == PICO4_LENS_NONE);
	EXPECT(NEAR(lens[0].f, 0.025) && lens[0].cx == 0 && lens[0].cy == 0 && NEAR(lens[1].f, 0));
	// A factory file with both polynomials reads as pico4_lens_parse_optical reads it, the only source before.
	struct pico4_lens before[2];
	EXPECT(pico4_lens_load(optical, NULL, NULL, lens, &loaded) && pico4_lens_parse_optical(optical, before));
	EXPECT(memcmp(lens, before, sizeof(lens)) == 0);

	// The rest of the file reads the factory fixture's lens again.
	EXPECT(pico4_lens_parse_optical(optical, lens));

	// s(rsq) = 1 + 0.5 rsq.
	EXPECT(NEAR(pico4_lens_scale(&lens[0], 2), 2));

	// Right eye: identity polynomial, so green = (x - cx) / F; red and blue scaled by 1.01 and 0.98.
	double t[3][2];
	EXPECT(pico4_lens_tan(&lens[1], &ca, 0.01, 0.002, t));
	EXPECT(NEAR(t[1][0], 0.4) && NEAR(t[1][1], (0.002 - 0.00025) / 0.025));
	EXPECT(NEAR(t[0][0], 0.4 * 1.01) && NEAR(t[2][0], 0.4 * 0.98));

	// The panel corner is outside the circle the polynomial was fitted for.
	EXPECT(pico4_lens_tan(&lens[1], &ca, 0.022, 0, t));
	EXPECT(!pico4_lens_tan(&lens[1], &ca, 0.022, 0.022, t));

	// The inverse finds the panel point back for every channel inside the fitted radius, with
	// the curved left lens and its centre offset.
	for (int e = 0; e < 2; e++) {
		for (int c = 0; c < 3; c++) {
			for (double x = -0.016; x <= 0.016; x += 0.004) {
				for (double y = -0.016; y <= 0.016; y += 0.004) {
					double fwd[3][2], bx, by;
					pico4_lens_tan(&lens[e], &ca, x, y, fwd);
					EXPECT(pico4_lens_inverse(&lens[e], &ca, c, fwd[c][0], fwd[c][1], &bx, &by));
					EXPECT(fabs(bx - x) < 1e-9 && fabs(by - y) < 1e-9);
				}
			}
		}
	}
	// Beyond it the inverse continues linearly: further out for larger angles, and an
	// 85-degree direction lands far outside the panel instead of just beyond its edge.
	double ix = 0, iy = 0, jx = 0, jy = 0;
	EXPECT(pico4_lens_inverse(&lens[1], &ca, 1, 1.0, 0, &ix, &iy) && pico4_lens_inverse(&lens[1], &ca, 1, 1.2, 0, &jx, &jy));
	EXPECT(jx > ix && ix > 0.0235);
	EXPECT(pico4_lens_inverse(&lens[1], &ca, 1, tan(85 * M_PI / 180), 0, &ix, &iy) && ix > 0.2);
	EXPECT(!pico4_lens_inverse(&lens[1], &ca, 3, 0, 0, &ix, &iy));

	// Field of view of the identity lens: atan(0.023 / 0.025) to each side, symmetric despite the
	// right lens centre's offset, as stock's.
	double l, r, u, d;
	pico4_lens_fov(&lens[1], &l, &r, &u, &d);
	EXPECT(NEAR(l, -atan(0.023 / 0.025)) && NEAR(r, atan(0.023 / 0.025)));
	EXPECT(NEAR(u, atan(0.023 / 0.025)) && NEAR(d, -atan(0.023 / 0.025)));

	// The only MCU request: A0 08 00 00 00 with its XOR checksum.
	EXPECT(pico4_ipd_get_position[0] == 0xA0 && pico4_ipd_get_position[1] == 0x08 &&
	       pico4_ipd_get_position[5] == 0xA8);
	// Reply: 79, position 6350 (63.50 mm) and ADC 1800 in either order, XOR checksum.
	double ipd = 0;
	uint8_t reply[6] = {0x79, 6350 & 0xff, 6350 >> 8, 1800 & 0xff, 1800 >> 8, 0};
	for (int i = 0; i < 5; i++) {
		reply[5] ^= reply[i];
	}
	EXPECT(pico4_ipd_parse_position(reply, sizeof(reply), &ipd) && NEAR(ipd, 0.0635));
	uint8_t swapped[6] = {0x79, 1800 & 0xff, 1800 >> 8, 6350 & 0xff, 6350 >> 8, 0};
	for (int i = 0; i < 5; i++) {
		swapped[5] ^= swapped[i];
	}
	EXPECT(pico4_ipd_parse_position(swapped, sizeof(swapped), &ipd) && NEAR(ipd, 0.0635));
	reply[5] ^= 1;
	EXPECT(!pico4_ipd_parse_position(reply, sizeof(reply), &ipd));

	// The port lock shared with pico4-fan: while another descriptor holds it, the read
	// gives up after a second; once it is free the read goes on (and a plain file then
	// fails at tcgetattr).
	char port[] = "test_pico4_ipd_XXXXXX";
	int holder = mkstemp(port);
	EXPECT(holder >= 0 && flock(holder, LOCK_EX) == 0);
	EXPECT(pico4_ipd_read(port, &ipd) == -EBUSY);
	close(holder);
	EXPECT(pico4_ipd_read(port, &ipd) == -ENOTTY);
	unlink(port);

	if (failures == 0) {
		printf("test_pico4_lens: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
