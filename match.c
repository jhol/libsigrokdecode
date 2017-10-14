/*
 * This file is part of the libsigrokdecode project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2012 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2017 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "libsigrokdecode-internal.h"

#include <glib.h>
#include <stdint.h>

/**
 * Check whether the specified sample matches the specified term.
 *
 * In the case of SRD_TERM_SKIP, this function can modify
 * term->num_samples_already_skipped.
 *
 * @param old_sample The value of the previous sample (0/1).
 * @param sample The value of the current sample (0/1).
 * @param term The term that should be checked for a match. Must not be NULL.
 *
 * @retval TRUE The current sample matches the specified term.
 * @retval FALSE The current sample doesn't match the specified term, or an
 *               invalid term was provided.
 *
 * @private
 */
static gboolean sample_matches(uint8_t old_sample, uint8_t sample, struct srd_term *term)
{
	/* Caller ensures term != NULL. */

	switch (term->type) {
	case SRD_TERM_HIGH:
		if (sample == 1)
			return TRUE;
		break;
	case SRD_TERM_LOW:
		if (sample == 0)
			return TRUE;
		break;
	case SRD_TERM_RISING_EDGE:
		if (old_sample == 0 && sample == 1)
			return TRUE;
		break;
	case SRD_TERM_FALLING_EDGE:
		if (old_sample == 1 && sample == 0)
			return TRUE;
		break;
	case SRD_TERM_EITHER_EDGE:
		if ((old_sample == 1 && sample == 0) || (old_sample == 0 && sample == 1))
			return TRUE;
		break;
	case SRD_TERM_NO_EDGE:
		if ((old_sample == 0 && sample == 0) || (old_sample == 1 && sample == 1))
			return TRUE;
		break;
	case SRD_TERM_SKIP:
		if (term->num_samples_already_skipped == term->num_samples_to_skip)
			return TRUE;
		term->num_samples_already_skipped++;
		break;
	default:
		srd_err("Unknown term type %d.", term->type);
		break;
	}

	return FALSE;
}

static gboolean have_non_null_conds(const struct srd_decoder_inst *di)
{
	GSList *l, *cond;

	if (!di)
		return FALSE;

	for (l = di->condition_list; l; l = l->next) {
		cond = l->data;
		if (cond)
			return TRUE;
	}

	return FALSE;
}

static void update_old_pins_array(struct srd_decoder_inst *di,
		const uint8_t *sample_pos)
{
	uint8_t sample;
	unsigned int i;
	int byte_offset, bit_offset;

	if (!di || !di->dec_channelmap || !sample_pos)
		return;

	for (i = 0; i < di->dec_num_channels; i++) {
		byte_offset = di->dec_channelmap[i] / 8;
		bit_offset = di->dec_channelmap[i] % 8;
		sample = *(sample_pos + byte_offset) & (1 << bit_offset) ? 1 : 0;
		di->old_pins_array->data[i] = sample;
	}
}

static void update_old_pins_array_initial_pins(struct srd_decoder_inst *di)
{
	uint8_t sample;
	unsigned int i;
	int byte_offset, bit_offset;
	const uint8_t *sample_pos;

	if (!di || !di->dec_channelmap)
		return;

	sample_pos = di->inbuf + ((di->abs_cur_samplenum - di->abs_start_samplenum) * di->data_unitsize);

	for (i = 0; i < di->dec_num_channels; i++) {
		if (di->old_pins_array->data[i] != SRD_INITIAL_PIN_SAME_AS_SAMPLE0)
			continue;
		byte_offset = di->dec_channelmap[i] / 8;
		bit_offset = di->dec_channelmap[i] % 8;
		sample = *(sample_pos + byte_offset) & (1 << bit_offset) ? 1 : 0;
		di->old_pins_array->data[i] = sample;
	}
}

static gboolean term_matches(const struct srd_decoder_inst *di,
		struct srd_term *term, const uint8_t *sample_pos)
{
	uint8_t old_sample, sample;
	int byte_offset, bit_offset, ch;

	/* Caller ensures di, di->dec_channelmap, term, sample_pos != NULL. */

	if (term->type == SRD_TERM_SKIP)
		return sample_matches(0, 0, term);

	ch = term->channel;
	byte_offset = di->dec_channelmap[ch] / 8;
	bit_offset = di->dec_channelmap[ch] % 8;
	sample = *(sample_pos + byte_offset) & (1 << bit_offset) ? 1 : 0;
	old_sample = di->old_pins_array->data[ch];

	return sample_matches(old_sample, sample, term);
}

static gboolean all_terms_match(const struct srd_decoder_inst *di,
		const GSList *cond, const uint8_t *sample_pos)
{
	const GSList *l;
	struct srd_term *term;

	/* Caller ensures di, cond, sample_pos != NULL. */

	for (l = cond; l; l = l->next) {
		term = l->data;
		if (!term_matches(di, term, sample_pos))
			return FALSE;
	}

	return TRUE;
}

static gboolean at_least_one_condition_matched(
		const struct srd_decoder_inst *di, unsigned int num_conditions)
{
	unsigned int i;

	/* Caller ensures di != NULL. */

	for (i = 0; i < num_conditions; i++) {
		if (di->match_array->data[i])
			return TRUE;
	}

	return FALSE;
}

static gboolean find_match(struct srd_decoder_inst *di)
{
	uint64_t i, j, num_samples_to_process;
	GSList *l, *cond;
	const uint8_t *sample_pos;
	unsigned int num_conditions;

	/* Caller ensures di != NULL. */

	/* Check whether the condition list is NULL/empty. */
	if (!di->condition_list) {
		srd_dbg("NULL/empty condition list, automatic match.");
		return TRUE;
	}

	/* Check whether we have any non-NULL conditions. */
	if (!have_non_null_conds(di)) {
		srd_dbg("Only NULL conditions in list, automatic match.");
		return TRUE;
	}

	num_samples_to_process = di->abs_end_samplenum - di->abs_cur_samplenum;
	num_conditions = g_slist_length(di->condition_list);

	/* di->match_array is NULL here. Create a new GArray. */
	di->match_array = g_array_sized_new(FALSE, TRUE, sizeof(gboolean), num_conditions);
	g_array_set_size(di->match_array, num_conditions);

	/* Sample 0: Set di->old_pins_array for SRD_INITIAL_PIN_SAME_AS_SAMPLE0 pins. */
	if (di->abs_cur_samplenum == 0)
		update_old_pins_array_initial_pins(di);

	for (i = 0; i < num_samples_to_process; i++, (di->abs_cur_samplenum)++) {

		sample_pos = di->inbuf + ((di->abs_cur_samplenum - di->abs_start_samplenum) * di->data_unitsize);

		/* Check whether the current sample matches at least one of the conditions (logical OR). */
		/* IMPORTANT: We need to check all conditions, even if there was a match already! */
		for (l = di->condition_list, j = 0; l; l = l->next, j++) {
			cond = l->data;
			if (!cond)
				continue;
			/* All terms in 'cond' must match (logical AND). */
			di->match_array->data[j] = all_terms_match(di, cond, sample_pos);
		}

		update_old_pins_array(di, sample_pos);

		/* If at least one condition matched we're done. */
		if (at_least_one_condition_matched(di, num_conditions))
			return TRUE;
	}

	return FALSE;
}

/**
 * Process available samples and check if they match the defined conditions.
 *
 * This function returns if there is an error, or when a match is found, or
 * when all samples have been processed (whether a match was found or not).
 * This function immediately terminates when the decoder's wait() method
 * invocation shall get terminated.
 *
 * @param di The decoder instance to use. Must not be NULL.
 * @param found_match Will be set to TRUE if at least one condition matched,
 *                    FALSE otherwise. Must not be NULL.
 *
 * @retval SRD_OK No errors occured, see found_match for the result.
 * @retval SRD_ERR_ARG Invalid arguments.
 *
 * @private
 */
SRD_PRIV int process_samples_until_condition_match(struct srd_decoder_inst *di, gboolean *found_match)
{
	if (!di || !found_match)
		return SRD_ERR_ARG;

	*found_match = FALSE;
	if (di->want_wait_terminate)
		return SRD_OK;

	/* Check if any of the current condition(s) match. */
	while (TRUE) {
		/* Feed the (next chunk of the) buffer to find_match(). */
		*found_match = find_match(di);

		/* Did we handle all samples yet? */
		if (di->abs_cur_samplenum >= di->abs_end_samplenum) {
			srd_dbg("Done, handled all samples (abs cur %" PRIu64
				" / abs end %" PRIu64 ").",
				di->abs_cur_samplenum, di->abs_end_samplenum);
			return SRD_OK;
		}

		/* If we didn't find a match, continue looking. */
		if (!(*found_match))
			continue;

		/* At least one condition matched, return. */
		return SRD_OK;
	}

	return SRD_OK;
}

