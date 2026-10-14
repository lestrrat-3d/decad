package decad

import (
	"context"
	"fmt"
	"math/big"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

// The census keeps a private deep copy of each arc it derived, and a consumer
// holding the census compares an owned arc with that copy instead of
// rederiving it. This test holds the comparison to the full live check for
// seven sample changes of an owned arc, and requires the comparison with the
// snapshot to refuse a change of each of the 19 rational leaves (in place and
// by replacement) and of every scalar and identity field the arc equality
// reads.
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

		// Every field the equality reads must refuse a change through the
		// snapshot comparison. The full check agrees on the samples above.
		refuses := func(name string, apply func() (restore func())) {
			restore := apply()
			in, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.arc)
			require.NoError(t, err, name)
			require.False(t, in, name)
			restore()
			in, err = revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.arc)
			require.NoError(t, err, name)
			require.True(t, in, name+" restored")
		}
		delta := big.NewRat(1, 1<<20)
		leaves := ratLeaves(own.arc)
		require.Len(t, leaves, 19)
		for i, leaf := range leaves {
			refuses(fmt.Sprintf("leaf %d in place", i), func() func() {
				saved := new(big.Rat).Set(*leaf)
				(*leaf).Add(*leaf, delta)
				return func() { (*leaf).Set(saved) }
			})
			refuses(fmt.Sprintf("leaf %d replaced", i), func() func() {
				saved := *leaf
				*leaf = new(big.Rat).Add(saved, delta)
				return func() { *leaf = saved }
			})
		}
		a := own.arc
		faces := trimmed.Faces()
		otherFace := func(f *Face) *Face {
			if f == faces[0] {
				return faces[1]
			}
			return faces[0]
		}
		scalars := map[string]func() func(){
			"band": func() func() { a.band++; return func() { a.band-- } },
			"through": func() func() {
				a.throughPositiveY = !a.throughPositiveY
				return func() { a.throughPositiveY = !a.throughPositiveY }
			},
			"blankFace": func() func() {
				f := a.circle.blankFace
				a.circle.blankFace = otherFace(f)
				return func() { a.circle.blankFace = f }
			},
			"capFace": func() func() {
				f := a.circle.capFace
				a.circle.capFace = otherFace(f)
				return func() { a.circle.capFace = f }
			},
			"capOrigin": func() func() {
				f := a.circle.capOrigin
				a.circle.capOrigin.Role += "x"
				return func() { a.circle.capOrigin = f }
			},
			"circle span": func() func() { a.circle.span++; return func() { a.circle.span-- } },
			"tool":        func() func() { a.circle.tool++; return func() { a.circle.tool-- } },
			"sourceMiss": func() func() {
				a.circle.sourceMiss = !a.circle.sourceMiss
				return func() { a.circle.sourceMiss = !a.circle.sourceMiss }
			},
		}
		for name, apply := range scalars {
			refuses(name, apply)
		}
		for bi, b := range [...]*revolveLoftFacesToolCapBoundary{&a.negativeZ, &a.positiveZ} {
			refuses(fmt.Sprintf("boundary %d segment", bi), func() func() { b.segment++; return func() { b.segment-- } })
			refuses(fmt.Sprintf("boundary %d span", bi), func() func() { b.span++; return func() { b.span-- } })
			refuses(fmt.Sprintf("boundary %d outward", bi), func() func() { b.outward = !b.outward; return func() { b.outward = !b.outward } })
			refuses(fmt.Sprintf("boundary %d ref", bi), func() func() {
				f := b.ref
				b.ref.Role += "x"
				return func() { b.ref = f }
			})
			refuses(fmt.Sprintf("boundary %d face", bi), func() func() {
				f := b.face
				b.face = otherFace(f)
				return func() { b.face = f }
			})
		}

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

// ratLeaves returns the address of every rational an occupied arc holds: the
// circle's x and radius, the time, and each boundary's parameter and box
// bounds.
func ratLeaves(a *revolveLoftFacesToolCapOccupiedArc) []**big.Rat {
	out := []**big.Rat{&a.circle.x, &a.circle.radius, &a.time}
	for _, b := range [...]*revolveLoftFacesToolCapBoundary{&a.negativeZ, &a.positiveZ} {
		out = append(out, &b.parameter.Lo, &b.parameter.Hi)
		for i := range b.box {
			out = append(out, &b.box[i].Lo, &b.box[i].Hi)
		}
	}
	return out
}
