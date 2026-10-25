package momentinput_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func momentCacheProfile() momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.NURBSSeg{Degree: 2, Control: []sectionrecord.Point2{{}, {U: 1, V: -1}, {U: 2}},
			Knots: []float64{0, 0, 0, 1, 1, 1}, Weights: []float64{1, 1, 1}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 2}, End: sectionrecord.Point2{}, TEnd: 1},
	}}}
}

func requireCachedRat(t *testing.T, want, got *big.Rat) {
	t.Helper()
	require.Equal(t, want == nil, got == nil)
	if want == nil {
		return
	}
	require.Zero(t, want.Num().Cmp(got.Num()), "numerator differs")
	require.Zero(t, want.Denom().Cmp(got.Denom()), "denominator differs")
}

func requireCachedIntegrals(t *testing.T, want, got momentinput.Integrals) {
	t.Helper()
	wantScalars := [13]float64{want.CoordUpper, want.Area, want.AreaBound, want.Mu, want.MuBound, want.Mv,
		want.MvBound, want.Muu, want.MuuBound, want.Muv, want.MuvBound, want.Mvv, want.MvvBound}
	gotScalars := [13]float64{got.CoordUpper, got.Area, got.AreaBound, got.Mu, got.MuBound, got.Mv,
		got.MvBound, got.Muu, got.MuuBound, got.Muv, got.MuvBound, got.Mvv, got.MvvBound}
	for i := range wantScalars {
		require.Equal(t, math.Float64bits(wantScalars[i]), math.Float64bits(gotScalars[i]), "scalar %d", i)
	}
	require.Equal(t, want.ExactDead, got.ExactDead)
	require.Equal(t, want.ThirdDead, got.ThirdDead)
	gotFields := got.Exact.Fields()
	for i, field := range want.Exact.Fields() {
		requireCachedRat(t, field, gotFields[i])
	}
	for i := range want.Third {
		requireCachedRat(t, want.Third[i].Lo, got.Third[i].Lo)
		requireCachedRat(t, want.Third[i].Hi, got.Third[i].Hi)
	}
}

func mutateCachedIntegrals(ig momentinput.Integrals) {
	for _, field := range ig.Exact.Fields() {
		if field != nil {
			field.SetInt64(-123)
		}
	}
	for _, interval := range ig.Third {
		if interval.Lo != nil {
			interval.Lo.SetInt64(-123)
		}
		if interval.Hi != nil {
			interval.Hi.SetInt64(-123)
		}
	}
}

func TestProfileMomentCacheOwnsRationalsAndSupportsConcurrentReads(t *testing.T) {
	t.Parallel()
	profile := momentCacheProfile()
	walks, err := momentinput.ResolveProfileWalks(profile, freeform.NewFreeformWork())
	require.NoError(t, err)
	work := freeform.NewFreeformWork()
	computed, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentThirdOrder, work)
	require.NoError(t, err)
	require.Zero(t, computed.Exact.Area.Cmp(big.NewRat(2, 3)))
	cache := walks.WithIntegrals(computed, freeform.MomentThirdOrder)
	require.True(t, cache != walks, "publication must not change an existing walk cache")
	mutateCachedIntegrals(computed)
	want, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentThirdOrder, freeform.NewFreeformWork())
	require.NoError(t, err)
	results := make([]momentinput.Integrals, 8)
	errs := make([]error, len(results))
	var group sync.WaitGroup
	for i := range results {
		group.Go(func() {
			results[i], errs[i] = cache.EvaluatorIntegralsUncheckedContext(t.Context(), profile,
				freeform.MomentThirdOrder, freeform.NewFreeformWork())
		})
	}
	group.Wait()
	for i, got := range results {
		require.NoError(t, errs[i])
		requireCachedIntegrals(t, want, got)
		mutateCachedIntegrals(got)
	}
	again, err := cache.EvaluatorIntegralsUncheckedContext(t.Context(), profile, freeform.MomentThirdOrder, freeform.NewFreeformWork())
	require.NoError(t, err)
	requireCachedIntegrals(t, want, again)
}

func TestProfileMomentCachePreservesPreflightAndFallbacks(t *testing.T) {
	t.Parallel()
	profile := momentCacheProfile()
	walks, err := momentinput.ResolveProfileWalks(profile, freeform.NewFreeformWork())
	require.NoError(t, err)
	work := freeform.NewFreeformWork()
	computed, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, work)
	require.NoError(t, err)
	charge := momentinput.WalkReadCharge{Spent: work.Spent, ReconstructionSpent: work.ReconstructionSpent}
	cache := walks.WithIntegrals(computed, freeform.MomentFirstOrder)
	unmetered := *cache
	unmetered.Metered = false
	workCases := []freeform.FreeformWork{
		{}, {Limit: 1}, {Limit: 32}, {Limit: 256}, {Limit: 1 << 20},
		{Limit: charge.Spent - 1}, {Limit: charge.Spent}, {Limit: charge.Spent + 1},
		{Limit: charge.Spent + 7, Spent: 8}, {Limit: charge.Spent + 7, Spent: 7},
		{ReconstructionLimit: charge.ReconstructionSpent - 1},
		{ReconstructionLimit: charge.ReconstructionSpent},
		{ReconstructionLimit: charge.ReconstructionSpent + 7, ReconstructionSpent: 8},
	}
	for cacheIndex, candidate := range []*momentinput.ProfileWalks{
		cache, walks, &unmetered, nil,
	} {
		for caseIndex, initial := range workCases {
			oldWork, newWork := initial, initial
			want, wantErr := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, &oldWork)
			got, gotErr := candidate.EvaluatorIntegralsUncheckedContext(t.Context(), profile, freeform.MomentFirstOrder, &newWork)
			require.Equal(t, oldWork, newWork, "cache %d case %d", cacheIndex, caseIndex)
			require.Equal(t, wantErr == nil, gotErr == nil, "cache %d case %d", cacheIndex, caseIndex)
			if wantErr == nil {
				requireCachedIntegrals(t, want, got)
				continue
			}
			require.EqualError(t, gotErr, wantErr.Error())
			var wantLimit, gotLimit *proofbound.WorkLimitError
			if errors.As(wantErr, &wantLimit) {
				require.ErrorAs(t, gotErr, &gotLimit)
				require.Equal(t, wantLimit.Resource, gotLimit.Resource)
				require.Equal(t, wantLimit.Maximum, gotLimit.Maximum)
				require.Equal(t, wantLimit.Observed, gotLimit.Observed)
			}
		}
	}
	wantNil, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, nil)
	require.NoError(t, err)
	gotNil, err := cache.EvaluatorIntegralsUncheckedContext(t.Context(), profile, freeform.MomentFirstOrder, nil)
	require.NoError(t, err)
	requireCachedIntegrals(t, wantNil, gotNil)
	wantSecond, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentSecondOrder,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	gotSecond, err := cache.EvaluatorIntegralsUncheckedContext(t.Context(), profile, freeform.MomentSecondOrder,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	requireCachedIntegrals(t, wantSecond, gotSecond)
	// Mutating an aliased source array must not alter the cache's identity key.
	curve := profile.Outer.Segments[0].(sectionrecord.NURBSSeg)
	curve.Control[1].V = math.Nextafter(curve.Control[1].V, math.Inf(-1))
	for _, order := range []freeform.MomentIntegralOrder{freeform.MomentFirstOrder, freeform.MomentSecondOrder} {
		want, err := profile.EvaluatorIntegralsUncheckedContext(t.Context(), order, freeform.NewFreeformWork())
		require.NoError(t, err)
		got, err := cache.EvaluatorIntegralsUncheckedContext(t.Context(), profile, order, freeform.NewFreeformWork())
		require.NoError(t, err)
		requireCachedIntegrals(t, want, got)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = cache.EvaluatorIntegralsUncheckedContext(ctx, profile, freeform.MomentFirstOrder, freeform.NewFreeformWork())
	require.ErrorIs(t, err, context.Canceled)
}
