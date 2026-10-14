package decad

import (
	"context"
	"math/big"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

// The census keeps a private deep copy of each arc it derived, and a consumer
// holding the census compares an owned arc with that copy instead of
// rederiving it. This test holds the comparison to the full live check for
// every change a consumer could make to an owned arc, in place or by
// replacement.
func TestCurvedCensusOwnedArcsMatchLiveCheck(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, census.ownArcs, 2)
	for _, own := range census.ownArcs {
		owners := 0
		for i := range census.pairs {
			if census.pairs[i].toolArc == own.arc {
				owners++
			}
		}
		require.Equal(t, 1, owners)
		require.NotSame(t, own.arc.time, own.derived.time)
	}

	other := trimmed.Faces()[0]
	type change struct {
		name    string
		apply   func(a *revolveLoftFacesToolCapOccupiedArc) (restore func())
		refused bool
	}
	changes := []change{
		{"unchanged", func(*revolveLoftFacesToolCapOccupiedArc) func() { return func() {} }, false},
		{"time replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.time
			a.time = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.time = saved }
		}, true},
		{"time in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := new(big.Rat).Set(a.time)
			a.time.Add(a.time, big.NewRat(1, 1<<20))
			return func() { a.time.Set(saved) }
		}, true},
		{"parameter replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.negativeZ.parameter.Lo
			a.negativeZ.parameter.Lo = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.negativeZ.parameter.Lo = saved }
		}, true},
		{"box in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			x := a.positiveZ.box[1].Hi
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"radius in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			x := a.circle.radius
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"through flipped", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			a.throughPositiveY = !a.throughPositiveY
			return func() { a.throughPositiveY = !a.throughPositiveY }
		}, true},
		{"face replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.negativeZ.face
			a.negativeZ.face = other
			if other == saved {
				a.negativeZ.face = trimmed.Faces()[1]
			}
			return func() { a.negativeZ.face = saved }
		}, true},
	}
	both := func(c *revolveLoftFacesPairCensus, arc *revolveLoftFacesToolCapOccupiedArc) (bool, bool) {
		in, inErr := revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, c, arc)
		full, fullErr := revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.Equal(t, fullErr, inErr)
		require.Equal(t, full, in)
		return in, full
	}
	for _, own := range census.ownArcs {
		for _, c := range changes {
			restore := c.apply(own.arc)
			got, _ := both(census, own.arc)
			require.Equal(t, !c.refused, got, c.name)
			both(nil, own.arc)
			restore()
		}
		got, _ := both(census, own.arc)
		require.True(t, got)

		// A deep copy is not owned and takes the full path.
		clone := revolveLoftFacesCloneToolCapOccupiedArc(own.arc)
		got, _ = both(census, &clone)
		require.True(t, got)
		clone.time.Add(clone.time, big.NewRat(1, 1<<20))
		got, _ = both(census, &clone)
		require.False(t, got)
	}

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	for _, own := range census.ownArcs {
		_, inErr := revolveLoftFacesToolCapOccupiedArcCurrentIn(cancelled, proof, census, own.arc)
		_, fullErr := revolveLoftFacesToolCapOccupiedArcCurrent(cancelled, proof, own.arc)
		require.ErrorIs(t, inErr, context.Canceled)
		require.ErrorIs(t, fullErr, context.Canceled)
	}

	require.Equal(t, 6, reflect.TypeFor[revolveLoftFacesToolCapOccupiedArc]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 8, reflect.TypeFor[revolveLoftFacesToolCapCircle]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 7, reflect.TypeFor[revolveLoftFacesToolCapBoundary]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
}
