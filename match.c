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

static gboolean match_skip_terms(struct srd_decoder_inst *di,
	const uint64_t num_samples_processed) {
	const GSList *l, *m;
	gboolean only_skip, all_skips_complete;
	for (l = di->condition_list; l; l = l->next) {
		all_skips_complete = TRUE;
		only_skip = TRUE;
		for (m = l->data; m; m = m->next) {
			struct srd_term *const term = m->data;
			if (term->type == SRD_TERM_SKIP) {
				term->num_samples_already_skipped +=
					num_samples_processed;
				if (term->num_samples_already_skipped <
					term->num_samples_to_skip)
					all_skips_complete = FALSE;
			} else {
				only_skip = FALSE;
			}

		}

		if (only_skip && all_skips_complete)
			return TRUE;
	}

	return FALSE;
}

#if defined(HAVE_ALIGNED_ACCESS_REQUIRED) && !defined(WORDS_BIGENDIAN)
#else
// SORT THIS OUT!!!
#endif

#define define_find_match_func(width, sample_type) 				\
										\
struct condition_masks_##width {						\
	sample_type high, low, change, no_change;				\
};										\
										\
static sample_type unpack_sample_##width(const uint8_t *ptr)			\
{										\
	return *(sample_type*)ptr;						\
}										\
										\
static void make_condition_masks_##width(struct srd_decoder_inst *const di,	\
	struct condition_masks_##width *const condition_masks_array,		\
	size_t *const condition_nums_array,					\
	uint64_t num_samples_to_process,					\
	size_t *const num_active_conditions,					\
	uint64_t *const num_applicable_samples)					\
{										\
	unsigned int condition_num;						\
	const GSList *l, *m;							\
	*num_active_conditions = 0;						\
	*num_applicable_samples = num_samples_to_process;			\
	for (l = di->condition_list, condition_num = 0; l;			\
		l = l->next, condition_num++) {					\
		struct condition_masks_##width *const masks =			\
			condition_masks_array + *num_active_conditions;		\
		for (m = l->data; m; m = m->next) {				\
			const struct srd_term *const term = m->data;		\
										\
			if (term->type == SRD_TERM_SKIP) {			\
				const uint64_t num_remaining_to_skip =		\
					term->num_samples_to_skip -		\
					term->num_samples_already_skipped;	\
				if (num_remaining_to_skip <			\
					*num_applicable_samples)		\
					*num_applicable_samples =		\
						num_remaining_to_skip;		\
				continue;					\
			}							\
										\
			const sample_type bit = 1ULL <<				\
				di->dec_channelmap[term->channel];		\
										\
			if (term->type == SRD_TERM_HIGH ||			\
				term->type == SRD_TERM_RISING_EDGE)		\
				masks->high |= bit;				\
			if (term->type == SRD_TERM_LOW ||			\
				term->type == SRD_TERM_FALLING_EDGE)		\
				masks->low |= bit;				\
			if (term->type == SRD_TERM_RISING_EDGE ||		\
				term->type == SRD_TERM_FALLING_EDGE ||		\
				term->type == SRD_TERM_EITHER_EDGE)		\
				masks->change |= bit;				\
			if (term->type == SRD_TERM_NO_EDGE)			\
				masks->no_change |= bit;			\
		}								\
										\
		if (masks->high || masks->low ||				\
			masks->change || masks->no_change) {			\
			condition_nums_array[*num_active_conditions] =		\
				condition_num;					\
			(*num_active_conditions)++;				\
		}								\
	}									\
}										\
										\
static gboolean match_condition_##width(					\
	const struct condition_masks_##width *const masks,			\
	const sample_type sample, const sample_type change)			\
{										\
	return (sample & masks->high) == masks->high &&				\
		(~sample & masks->low) == masks->low &&				\
		(change & masks->change) == masks->change &&			\
		(~change & masks->no_change) == masks->no_change;		\
}										\
										\
static gboolean quick_search_matches_##width(					\
	const struct condition_masks_##width *const masks,			\
	const unsigned int num_active_conditions,				\
	const uint8_t **const sample_pos, const size_t num_samples,		\
	const size_t unitsize, sample_type *prev_sample, sample_type *sample,	\
	sample_type *change)							\
{										\
	unsigned int i;								\
	const uint8_t *const end_sample_pos = *sample_pos +			\
		num_samples * unitsize;						\
	while (*sample_pos < end_sample_pos) {					\
		*prev_sample = *sample;						\
		*sample = unpack_sample_##width(*sample_pos);			\
		*change = *sample ^ *prev_sample;				\
										\
		for (i = 0; i != num_active_conditions; i++)			\
			if (G_UNLIKELY (match_condition_##width(		\
				masks + i, *sample, *change)))			\
				return TRUE;					\
										\
		*sample_pos += unitsize;					\
	}									\
										\
	return FALSE;								\
}										\
										\
static gboolean find_match_##width(struct srd_decoder_inst *di)			\
{										\
	gboolean any_match = FALSE;						\
	size_t num_active_conditions;						\
	uint64_t num_applicable_samples;					\
	uint64_t num_samples_to_process =					\
		di->abs_end_samplenum - di->abs_cur_samplenum;			\
	sample_type prev_sample = 0, sample = 0, change = 0;			\
	unsigned int i;								\
										\
	const unsigned int unitsize = di->data_unitsize;			\
	const unsigned int num_conditions = g_slist_length(di->condition_list);	\
	const uint8_t *sample_pos = di->inbuf + unitsize *			\
		(di->abs_cur_samplenum - di->abs_start_samplenum);		\
										\
	/* Allocate mask buffers. */						\
	struct condition_masks_##width *const masks =				\
		g_malloc0_n(num_conditions,					\
			sizeof(struct condition_masks_##width));		\
	size_t *const mask_condition_nums =					\
		g_malloc_n(num_conditions, sizeof(size_t));			\
										\
	/* di->match_array is NULL here. Create a new GArray. */		\
	di->match_array = g_array_sized_new(					\
		FALSE, TRUE, sizeof(gboolean), num_conditions);			\
	g_array_set_size(di->match_array, num_conditions);			\
										\
	/* Sample 0: Set di->old_pins_array for SRD_INITIAL_PIN_SAME_AS_SAMPLE0 \
	 * pins. */								\
	if (di->abs_cur_samplenum == 0)						\
		update_old_pins_array_initial_pins(di);				\
										\
	for (i = 0; i != di->dec_num_channels; i++)				\
		if (di->old_pins_array->data[i])				\
			sample |= 1ULL << di->dec_channelmap[i];		\
										\
	while (num_samples_to_process && !any_match) {				\
		make_condition_masks_##width(di, masks, mask_condition_nums,	\
			num_samples_to_process, &num_active_conditions,		\
			&num_applicable_samples);				\
										\
		const uint8_t *const start_sample_pos = sample_pos;		\
		if (num_active_conditions) {					\
			/* Search for any matching conditions */		\
			any_match = quick_search_matches_##width(masks,		\
				num_active_conditions,	&sample_pos,		\
				num_applicable_samples, unitsize,		\
				&prev_sample, &sample, &change);		\
		} else if (num_applicable_samples) {				\
			/* If there are no active masks i.e. only skip		\
			 * conditions, skip ahead by pointer arithmetic, and	\
			 * update the the sample words on the final samples.	\
			 */							\
			sample_pos += (num_applicable_samples - 1) * unitsize;	\
			prev_sample = unpack_sample_##width(sample_pos);	\
			sample_pos += unitsize;					\
			sample = unpack_sample_##width(sample_pos);		\
			change = sample ^ prev_sample;				\
		}								\
										\
		const uint64_t num_samples_processed =				\
			(sample_pos - start_sample_pos) / unitsize;		\
		num_samples_to_process -= num_samples_processed;		\
		any_match |= match_skip_terms(di, num_samples_processed);	\
	}									\
										\
	/* If any matched recalculate the results for the match array */	\
	if (any_match) {							\
		for (i = 0; i != num_active_conditions; i++) {			\
			di->match_array->data[mask_condition_nums[i]] =		\
				match_condition_##width(			\
					masks + i, sample, change);		\
		}								\
	}									\
										\
	g_free(masks);								\
	g_free(mask_condition_nums);						\
										\
	di->abs_cur_samplenum = (sample_pos - di->inbuf) / unitsize +		\
		di->abs_start_samplenum;					\
	update_old_pins_array(di, sample_pos);					\
										\
	return any_match;							\
}

define_find_match_func(8, uint8_t)
define_find_match_func(16, uint16_t)
define_find_match_func(32, uint32_t)
define_find_match_func(64, uint64_t)

static gboolean find_match(struct srd_decoder_inst *di)
{
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

	switch (di->data_unitsize) {
	case 1: return find_match_8(di);
	case 2: return find_match_16(di);
	case 4: return find_match_32(di);
	case 8: return find_match_64(di);
	default:
		assert(FALSE);
	}
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
	while (!*found_match) {
		/* Did we handle all samples yet? */
		if (di->abs_cur_samplenum >= di->abs_end_samplenum) {
			srd_dbg("Done, handled all samples (abs cur %" PRIu64
				" / abs end %" PRIu64 ").",
				di->abs_cur_samplenum, di->abs_end_samplenum);
			return SRD_OK;
		}

		/* Feed the (next chunk of the) buffer to find_match(). */
		*found_match = find_match(di);
	}

	return SRD_OK;
}
