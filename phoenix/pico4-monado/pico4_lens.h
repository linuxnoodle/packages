// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 lens model: per-eye radial polynomial with chromatic aberration terms.
 *
 * The factory file OpticalCalibrationData.json gives, per eye, the scale F
 * (metres on the panel per tan-angle at the centre) and coefficients K0..K10 of
 * a polynomial in r², or names the lens's batch, whose profile the stock table
 * of lens batches gives (pico4_lens_load). A panel point p (metres from the lens
 * centre) is seen
 * at tan-angle t·s(|t|²) with t = p / F and s(x) = Σ Kᵢ xⁱ. Red and blue scale
 * that by (1 + CA0 + CA1·r² + CA4·r) and (1 + CA2 + CA3·r² + CA5·r); the six CA
 * terms come from the stock lens configuration. docs/xr.md has the details.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_LENS_COEFFS 11

//! Width and height of one eye's panel in metres (2160 px at 21.3 µm).
#define PICO4_LENS_EYE_M 0.046

//! Room for a lens serial number, "m0B_h3_d20220815" (pico4_lens_parse_serial), and for a longer one.
#define PICO4_LENS_SERIAL_MAX 64

struct pico4_lens
{
	double f;                    //!< Metres per tan-angle at the lens centre.
	double k[PICO4_LENS_COEFFS]; //!< Polynomial in r².
	double cx, cy;               //!< Lens centre on the panel, metres from its centre in compute_distortion's u and v.
};

struct pico4_lens_ca
{
	double ca[6];
};

//! The file an eye's lens profile came from (pico4_lens_loaded).
enum pico4_lens_source
{
	PICO4_LENS_NONE = 0, //!< No file gives the eye a profile: no distortion correction.
	PICO4_LENS_FACTORY,  //!< The factory file's distortion_parameter.
	PICO4_LENS_BATCH,    //!< The lens's batch in the table of lens batches.
	PICO4_LENS_GENERIC,  //!< The stock lens configuration's generic profile.
};

/*!
 * Parse one eye's (0 left, 1 right) polynomial from OpticalCalibrationData.json text into
 * *out_lens: F and A0..A10, the lens centre as it is. False when the file names no complete
 * polynomial for the eye; *out_lens is then as it was.
 */
bool
pico4_lens_parse_factory(const char *json_text, int eye, struct pico4_lens *out_lens);

/*!
 * Parse both eyes (0 left, 1 right) from OpticalCalibrationData.json text.
 * The lens centre offsets (lens_center_gos, mm) are optional.
 */
bool
pico4_lens_parse_optical(const char *json_text, struct pico4_lens out_lenses[2]);

/*!
 * Parse only the lens centre offsets (lens_center_gos, mm) of OpticalCalibrationData.json
 * text into the lenses, the same optional fields pico4_lens_parse_optical applies; the
 * polynomial is left as it is. False when the file names no offsets at all.
 */
bool
pico4_lens_parse_centers(const char *json_text, struct pico4_lens in_out_lenses[2]);

/*!
 * The lens serial number of eye (0 left, 1 right) from the factory file's lens_data, as
 * PICO OS reads it: "m<lens_model>_h<lens_hole>_d<lens_date>", the mould, the cavity and the
 * date of the lens's batch. False when the file names no lens for the eye or a part does not
 * fit; *out_serial is then empty.
 */
bool
pico4_lens_parse_serial(const char *json_text, int eye, char out_serial[PICO4_LENS_SERIAL_MAX]);

/*!
 * PICO OS's pick for a lens serial number in the table of lens batches
 * (pxrconfig/SDK_Lens_Distortion_MatchConfigFile.txt, JSON text), into *out_lens: F and
 * A0..A10 of the picked entry's profile, the lens centre as it is. Of the entries whose serial
 * number contains the lens's mould and cavity ("<model>_h<hole>") and the table's "default"
 * entry, the one that sorts last at or below the lens's serial number, by plain string order:
 * the newest batch of that mould and cavity dated on or before the lens, "default" for a mould
 * or cavity the table does not list and for a lens older than every batch. False when no such
 * entry gives F and eleven coefficients; *out_lens is then as it was.
 */
bool
pico4_lens_parse_match_table(const char *json_text, const char *serial, struct pico4_lens *out_lens);

/*!
 * Parse the stock lens configuration (SDK_Lens_ConfigFile.txt, JSON text) into *out_lens: its
 * generic profile, the one PICO OS runs for an eye that neither the factory file nor the table
 * of lens batches gives one. Its metres per tan-angle at the centre is the profile's F and its
 * distortion coefficients the same power series in r² as the factory file's A0..A10, in the
 * equation type every PICO 4 build ships; another equation type is refused rather than
 * misread. The lens centre stays as it is.
 */
bool
pico4_lens_parse_sdk_table(const char *json_text, struct pico4_lens *out_lens);

//! What pico4_lens_load found.
struct pico4_lens_loaded
{
	enum pico4_lens_source source[2];        //!< Where each eye's (0 left, 1 right) profile came from.
	char serial[2][PICO4_LENS_SERIAL_MAX]; //!< Each eye's lens serial number, empty when the factory file names none.
};

/*!
 * Both eyes' lens models into *out_lenses from the factory file (OpticalCalibrationData.json
 * text), the table of lens batches (pxrconfig/SDK_Lens_Distortion_MatchConfigFile.txt text)
 * and the stock lens configuration (SDK_Lens_ConfigFile.txt text), any of which may be NULL.
 * Per eye, as PICO OS picks it: F and A0..A10 of the factory file's polynomial; else of the
 * lens's batch, which the factory file names by its serial number; else of the stock lens
 * configuration's generic profile. Each profile is whole: F comes from where the coefficients
 * come. Each eye's source and serial number go to *out_loaded. True when both eyes have a
 * profile: their models are then whole, the lens centre at the panel's centre, moved by the
 * factory file's offsets (pico4_lens_parse_centers). False when an eye has none: that eye's
 * model is all zero, the other's is its profile's with its centre at the panel's centre.
 */
bool
pico4_lens_load(const char *factory,
                const char *table,
                const char *config,
                struct pico4_lens out_lenses[2],
                struct pico4_lens_loaded *out_loaded);

//! Parse the six chromatic aberration terms from the stock lens configuration (JSON text).
bool
pico4_lens_parse_ca(const char *json_text, struct pico4_lens_ca *out_ca);

//! s(rsq) = Σ Kᵢ rsqⁱ.
double
pico4_lens_scale(const struct pico4_lens *lens, double rsq);

/*!
 * Tan-angles seen through the lens at panel point (x, y), metres from the panel
 * centre (x right, y down), for red, green and blue: out[channel][0 = x, 1 = y].
 * Returns false outside the radius the polynomial was fitted for (the panel's
 * inscribed circle); there the tan-angles grow steeply from their value on that
 * circle, so the image ends smoothly at the edge of the rendered view.
 */
bool
pico4_lens_tan(const struct pico4_lens *lens, const struct pico4_lens_ca *ca, double x, double y, double out[3][2]);

/*!
 * The inverse of pico4_lens_tan for one channel (0 red, 1 green, 2 blue): the panel
 * point (metres from the panel centre, x right, y down) seen at tan-angles (tx, ty).
 * The model is radial about the lens centre, so the point lies on the ray from the
 * centre in the direction of (tx, ty); within the fitted radius its distance is found
 * by bisection. Beyond it the inverse continues at its slope there rather than
 * inverting the steep continuation of pico4_lens_tan, so directions far outside the
 * view land far outside the panel. False for an unknown channel.
 */
bool
pico4_lens_inverse(const struct pico4_lens *lens,
                   const struct pico4_lens_ca *ca,
                   int channel,
                   double tx,
                   double ty,
                   double *out_x,
                   double *out_y);

/*!
 * Field of view of one eye in radians, from the panel centre to its edges through
 * the green channel, symmetric whatever the lens centre's offset (as stock's):
 * left and down negative, as in xrt_fov.
 */
void
pico4_lens_fov(const struct pico4_lens *lens,
               double *out_left,
               double *out_right,
               double *out_up,
               double *out_down);

#ifdef __cplusplus
}
#endif
