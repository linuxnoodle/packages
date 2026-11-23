// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 lens model (see pico4_lens.h).
 * @ingroup drv_pico4
 */

#include "pico4_lens.h"

#include <cjson/cJSON.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//! The polynomials are fitted inside the panel's inscribed circle; they diverge in the corners.
#define PICO4_LENS_FIT_RADIUS_M (PICO4_LENS_EYE_M / 2 * 1.02)

//! Numbers in these files are strings ("0.025000"); accept plain numbers too.
static bool
get_number(const cJSON *obj, const char *name, double *out)
{
	const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
	char *end = NULL;
	if (cJSON_IsNumber(item)) {
		*out = item->valuedouble;
		return true;
	}
	if (cJSON_IsString(item) && item->valuestring != NULL) {
		*out = strtod(item->valuestring, &end);
		return end != item->valuestring;
	}
	return false;
}

//! The eye's (0 left, 1 right) name in the factory file; no eye's for any other number.
static const char *
eye_name(int eye)
{
	static const char *eyes[2] = {"left", "right"};
	return eye == 0 || eye == 1 ? eyes[eye] : "";
}

bool
pico4_lens_parse_factory(const char *json_text, int eye, struct pico4_lens *out_lens)
{
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "distortion_parameter");
	const cJSON *p = cJSON_GetObjectItemCaseSensitive(params, eye_name(eye));
	struct pico4_lens lens = {0};
	bool ok = get_number(p, "F", &lens.f) && lens.f > 0;
	for (int i = 0; ok && i < PICO4_LENS_COEFFS; i++) {
		char name[8];
		snprintf(name, sizeof(name), "A%d", i);
		ok = get_number(p, name, &lens.k[i]);
	}
	cJSON_Delete(root);
	if (ok) {
		out_lens->f = lens.f;
		memcpy(out_lens->k, lens.k, sizeof(lens.k));
	}
	return ok;
}

bool
pico4_lens_parse_optical(const char *json_text, struct pico4_lens out_lenses[2])
{
	bool ok = true;
	for (int e = 0; ok && e < 2; e++) {
		out_lenses[e] = (struct pico4_lens){0};
		ok = pico4_lens_parse_factory(json_text, e, &out_lenses[e]);
	}
	if (ok) {
		(void)pico4_lens_parse_centers(json_text, out_lenses);
	}
	return ok;
}

bool
pico4_lens_parse_centers(const char *json_text, struct pico4_lens in_out_lenses[2])
{
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *center = cJSON_GetObjectItemCaseSensitive(root, "lens_center_gos");
	bool ok = center != NULL;

	/*
	 * The stock compositor's distortion mesh (libpxrcompositor.so) moves the lens centre
	 * to the wearer's left as centerx grows and up as centery grows, in both eyes, by
	 * 1 mm per mm. In compute_distortion's panel coordinates (x right, y down) that is
	 * cx = -centerx, cy = -centery. The offset moves only the lens model, as in stock:
	 * the field of view stays symmetric (pico4_lens_fov).
	 */
	for (int e = 0; ok && e < 2; e++) {
		double mm;
		if (get_number(cJSON_GetObjectItemCaseSensitive(center, "centerx"), eye_name(e), &mm)) {
			in_out_lenses[e].cx = -mm * 0.001;
		}
		if (get_number(cJSON_GetObjectItemCaseSensitive(center, "centery"), eye_name(e), &mm)) {
			in_out_lenses[e].cy = -mm * 0.001;
		}
	}
	cJSON_Delete(root);
	return ok;
}

//! A part of a serial number: the string as it is written, or a number.
static bool
get_text(const cJSON *obj, const char *name, char *out, size_t size)
{
	const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
	int n = -1;
	if (cJSON_IsString(item) && item->valuestring != NULL) {
		n = snprintf(out, size, "%s", item->valuestring);
	} else if (cJSON_IsNumber(item)) {
		n = snprintf(out, size, "%.17g", item->valuedouble);
	}
	return n > 0 && n < (int)size;
}

bool
pico4_lens_parse_serial(const char *json_text, int eye, char out_serial[PICO4_LENS_SERIAL_MAX])
{
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "lens_data");
	const cJSON *lens = cJSON_GetObjectItemCaseSensitive(data, eye_name(eye));
	char model[32], hole[32], date[32];
	int n = 0;
	if (get_text(lens, "lens_model", model, sizeof(model)) && get_text(lens, "lens_hole", hole, sizeof(hole)) &&
	    get_text(lens, "lens_date", date, sizeof(date))) {
		n = snprintf(out_serial, PICO4_LENS_SERIAL_MAX, "m%s_h%s_d%s", model, hole, date);
	}
	cJSON_Delete(root);
	if (n <= 0 || n >= PICO4_LENS_SERIAL_MAX) {
		out_serial[0] = '\0';
		return false;
	}
	return true;
}

//! F,A0,...,A10 as one profile into *out_lens, the lens centre as it is; false for anything else.
static bool
get_profile(const char *text, struct pico4_lens *out_lens)
{
	struct pico4_lens lens = {0};
	const char *s = text;
	bool ok = true;
	for (int i = 0; ok && i < PICO4_LENS_COEFFS + 1; i++) {
		char *end = NULL;
		double value = strtod(s, &end);
		ok = end != s;
		s = end;
		if (i == 0) {
			lens.f = value;
		} else {
			lens.k[i - 1] = value;
		}
		if (ok && i < PICO4_LENS_COEFFS) {
			ok = *s++ == ',';
		}
	}
	ok = ok && *s == '\0' && lens.f > 0;
	if (ok) {
		out_lens->f = lens.f;
		memcpy(out_lens->k, lens.k, sizeof(lens.k));
	}
	return ok;
}

bool
pico4_lens_parse_match_table(const char *json_text, const char *serial, struct pico4_lens *out_lens)
{
	/*
	 * The lens's mould and cavity, "0B_h3" of "m0B_h3_d20220815": from the second character up to the
	 * serial number's second underscore.
	 */
	const char *underscore = strchr(serial, '_');
	const char *second = underscore == NULL ? NULL : strchr(underscore + 1, '_');
	char mould[PICO4_LENS_SERIAL_MAX];
	ptrdiff_t length = second == NULL ? 0 : second - serial - 1;
	if (length <= 0 || length >= (ptrdiff_t)sizeof(mould)) {
		return false;
	}
	memcpy(mould, serial + 1, (size_t)length);
	mould[length] = '\0';

	cJSON *root = cJSON_Parse(json_text);
	const cJSON *entries = cJSON_GetObjectItemCaseSensitive(root, "ruleData");
	const cJSON *entry, *profile = NULL;
	const char *best = NULL;
	/*
	 * "default" sorts before every serial number, which starts with "m": it is always at or below the
	 * lens and loses to every batch of the lens's mould and cavity that is.
	 */
	cJSON_ArrayForEach(entry, entries)
	{
		const cJSON *key = cJSON_GetObjectItemCaseSensitive(entry, "key");
		const cJSON *value = cJSON_GetObjectItemCaseSensitive(entry, "value");
		const cJSON *linkage = cJSON_GetObjectItemCaseSensitive(entry, "linkageKey");
		if (!cJSON_IsString(key) || strcmp(key->valuestring, "lens_sn") != 0 ||
		    !cJSON_IsString(value) || !cJSON_IsString(linkage) ||
		    strcmp(linkage->valuestring, "sdk_distortion_para") != 0) {
			continue;
		}
		const char *name = value->valuestring;
		if ((strcmp(name, "default") == 0 || strstr(name, mould) != NULL) && strcmp(name, serial) <= 0 &&
		    (best == NULL || strcmp(name, best) > 0)) {
			best = name;
			profile = cJSON_GetObjectItemCaseSensitive(entry, "linkageValue");
		}
	}
	bool ok = cJSON_IsString(profile) && get_profile(profile->valuestring, out_lens);
	cJSON_Delete(root);
	return ok;
}

bool
pico4_lens_parse_sdk_table(const char *json_text, struct pico4_lens *out_lens)
{
	/*
	 * The stock lens configuration carries one generic profile. Its eleven distortion coefficients
	 * are the same power series in r² as the factory file's A0..A10 in equation type 2, the only one
	 * any PICO 4 build ships; another type's coefficients are refused rather than read as what they
	 * are not.
	 */
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *entries = cJSON_GetObjectItemCaseSensitive(root, "configData");
	struct pico4_lens lens = {0};
	double f = 0, eqn = 0;
	unsigned found = 0;
	const cJSON *entry;
	cJSON_ArrayForEach(entry, entries)
	{
		const cJSON *key = cJSON_GetObjectItemCaseSensitive(entry, "keyName");
		int index;
		char tail;
		double value;
		if (!cJSON_IsString(key) || !get_number(entry, "keyValue", &value)) {
			continue;
		}
		if (strcmp(key->valuestring, "sdk_lens_metersPerTanAngleAtCenter") == 0) {
			f = value;
		} else if (strcmp(key->valuestring, "sdk_lens_distortionEqnType") == 0) {
			eqn = value;
		} else if (sscanf(key->valuestring, "sdk_lens_distortionCoefficients%d%c", &index, &tail) == 1 &&
		           index >= 0 && index < PICO4_LENS_COEFFS) {
			lens.k[index] = value;
			found |= 1u << index;
		}
	}
	bool ok = f > 0 && eqn == 2 && found == (1u << PICO4_LENS_COEFFS) - 1;
	if (ok) {
		out_lens->f = f;
		memcpy(out_lens->k, lens.k, sizeof(lens.k));
	}
	cJSON_Delete(root);
	return ok;
}

/*!
 * The profile of eye (0 left, 1 right) into *out_lens as PICO OS picks it; the lens centre stays
 * as it is. Any text may be NULL. PICO4_LENS_NONE when no file gives the eye one.
 */
static enum pico4_lens_source
pick_profile(const char *factory,
     const char *table,
     const char *config,
     int eye,
     struct pico4_lens *out_lens,
     char out_serial[PICO4_LENS_SERIAL_MAX])
{
	out_serial[0] = '\0';
	/*
	 * PICO OS's config server (ConfigServerService) picks each eye's profile on its own: the
	 * factory file's polynomial; else the lens's batch, which the factory file names by the lens's
	 * serial number and the table of lens batches gives a profile; else what the stock lens
	 * configuration holds for every eye. Factory files exist without a polynomial, and one
	 * headset's two eyes carry lenses from different batches.
	 */
	if (factory != NULL && pico4_lens_parse_factory(factory, eye, out_lens)) {
		return PICO4_LENS_FACTORY;
	}
	if (factory != NULL && pico4_lens_parse_serial(factory, eye, out_serial) && table != NULL &&
	    pico4_lens_parse_match_table(table, out_serial, out_lens)) {
		return PICO4_LENS_BATCH;
	}
	if (config != NULL && pico4_lens_parse_sdk_table(config, out_lens)) {
		return PICO4_LENS_GENERIC;
	}
	return PICO4_LENS_NONE;
}

bool
pico4_lens_load(const char *factory,
                const char *table,
                const char *config,
                struct pico4_lens out_lenses[2],
                struct pico4_lens_loaded *out_loaded)
{
	bool ok = true;
	for (int e = 0; e < 2; e++) {
		out_lenses[e] = (struct pico4_lens){0};
		out_loaded->source[e] = pick_profile(factory, table, config, e, &out_lenses[e], out_loaded->serial[e]);
		ok = ok && out_loaded->source[e] != PICO4_LENS_NONE;
	}
	// The factory file's offsets go with the models, as they went with its polynomials: with a model for each eye.
	if (ok && factory != NULL) {
		(void)pico4_lens_parse_centers(factory, out_lenses);
	}
	return ok;
}

bool
pico4_lens_parse_ca(const char *json_text, struct pico4_lens_ca *out_ca)
{
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *entries = cJSON_GetObjectItemCaseSensitive(root, "configData");
	unsigned found = 0;
	const cJSON *entry;
	cJSON_ArrayForEach(entry, entries)
	{
		const cJSON *key = cJSON_GetObjectItemCaseSensitive(entry, "keyName");
		int index;
		char tail;
		double value;
		if (cJSON_IsString(key) &&
		    sscanf(key->valuestring, "sdk_lens_chromaticAberration%d%c", &index, &tail) == 1 && index >= 0 &&
		    index < 6 && get_number(entry, "keyValue", &value)) {
			out_ca->ca[index] = value;
			found |= 1u << index;
		}
	}
	cJSON_Delete(root);
	return found == 0x3f;
}

double
pico4_lens_scale(const struct pico4_lens *lens, double rsq)
{
	double s = 0;
	for (int i = PICO4_LENS_COEFFS - 1; i >= 0; i--) {
		s = s * rsq + lens->k[i];
	}
	return s;
}

bool
pico4_lens_tan(const struct pico4_lens *lens, const struct pico4_lens_ca *ca, double x, double y, double out[3][2])
{
	double px = x - lens->cx, py = y - lens->cy;
	double rm = sqrt(px * px + py * py);
	/*
	 * Beyond the fitted radius, evaluate the polynomial at that radius and let
	 * the tan-angle grow steeply instead: the image then ends smoothly where it
	 * leaves the rendered field of view, not at mesh-cell steps.
	 */
	double outside = rm > PICO4_LENS_FIT_RADIUS_M ? rm / PICO4_LENS_FIT_RADIUS_M : 1.0;
	double tx = px / outside / lens->f, ty = py / outside / lens->f;
	double rsq = tx * tx + ty * ty, r = sqrt(rsq);
	double s = pico4_lens_scale(lens, rsq) * outside * outside * outside * outside;
	const double *c = ca->ca;
	double scale[3] = {
	    s * (1 + c[0] + c[1] * rsq + c[4] * r),
	    s,
	    s * (1 + c[2] + c[3] * rsq + c[5] * r),
	};
	for (int ch = 0; ch < 3; ch++) {
		out[ch][0] = tx * scale[ch];
		out[ch][1] = ty * scale[ch];
	}
	return outside == 1.0;
}

bool
pico4_lens_inverse(const struct pico4_lens *lens,
                   const struct pico4_lens_ca *ca,
                   int channel,
                   double tx,
                   double ty,
                   double *out_x,
                   double *out_y)
{
	double want = sqrt(tx * tx + ty * ty);
	if (channel < 0 || channel > 2) {
		return false;
	}
	if (want == 0) {
		*out_x = lens->cx;
		*out_y = lens->cy;
		return true;
	}
	double dx = tx / want, dy = ty / want;
	double rf = PICO4_LENS_FIT_RADIUS_M, t[3][2];
	pico4_lens_tan(lens, ca, lens->cx + dx * rf, lens->cy + dy * rf, t);
	double tf = hypot(t[channel][0], t[channel][1]);
	if (want > tf) {
		/*
		 * Beyond the fitted radius the forward model grows steeply so that the image ends
		 * cleanly; inverted, that would squeeze every direction up to 90 degrees off-axis into
		 * a band just outside the panel, and geometry drawn through the inverse (SteamVR's
		 * Facet renderer) would leave stretched triangles along the edge instead of being
		 * clipped. The inverse continues at the slope it has at the fitted radius instead.
		 */
		double rin = rf * (1 - 1e-4);
		pico4_lens_tan(lens, ca, lens->cx + dx * rin, lens->cy + dy * rin, t);
		double slope = (tf - hypot(t[channel][0], t[channel][1])) / (rf - rin);
		if (!(slope > 0)) {
			return false;
		}
		double r = rf + (want - tf) / slope;
		*out_x = lens->cx + dx * r;
		*out_y = lens->cy + dy * r;
		return true;
	}
	double lo = 0, hi = rf;
	// The tan-angle grows with the distance from the centre: 50 halvings reach far below a nanometre.
	for (int i = 0; i < 50; i++) {
		double mid = (lo + hi) / 2;
		pico4_lens_tan(lens, ca, lens->cx + dx * mid, lens->cy + dy * mid, t);
		if (hypot(t[channel][0], t[channel][1]) < want) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	double r = (lo + hi) / 2;
	*out_x = lens->cx + dx * r;
	*out_y = lens->cy + dy * r;
	return true;
}

//! Green tan-angle at distance d (metres) from the lens centre along an axis.
static double
edge_tan(const struct pico4_lens *lens, double d)
{
	double t = d / lens->f;
	return t * pico4_lens_scale(lens, t * t);
}

void
pico4_lens_fov(const struct pico4_lens *lens, double *out_left, double *out_right, double *out_up, double *out_down)
{
	/*
	 * Symmetric, from the panel centre, whatever the lens centre's offset: stock renders each
	 * eye with one symmetric frustum and moves the lens centre only in its distortion mesh. On
	 * SteamVR an asymmetric vertical field of view moved the picture by its asymmetry, several
	 * times the offset itself (2 mm: 470 px instead of 94 on the panel).
	 */
	double a = atan(edge_tan(lens, PICO4_LENS_EYE_M / 2));
	*out_left = -a;
	*out_right = a;
	*out_up = a;
	*out_down = -a;
}
