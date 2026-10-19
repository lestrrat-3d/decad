package loftcylinder_test

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMirrorBlank(t *testing.T) {
	xy, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	blank := func(edit func(*prismrecord.Payload)) prismrecord.Payload {
		pp := prismrecord.Payload{Frame: xy, Z0: 0, Z1: 2, Xform: r3.Identity()}
		pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(4), CCW: true},
		}
		if edit != nil {
			edit(&pp)
		}
		return pp
	}
	require.True(t, loftcylinder.MirrorBlank(blank(nil)))
	for name, edit := range map[string]func(*prismrecord.Payload){
		"surface result": func(pp *prismrecord.Payload) { pp.SurfaceResult = true },
		"placed": func(pp *prismrecord.Payload) {
			pp.Xform = r3.Transform{}
		},
		"cap delta":     func(pp *prismrecord.Payload) { pp.Z1Delta = 1e-9 },
		"section delta": func(pp *prismrecord.Payload) { pp.SectionDelta = 1e-9 },
		"empty height":  func(pp *prismrecord.Payload) { pp.Z1 = pp.Z0 },
		"off axis": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{
				Center: sectionrecord.Point2{U: 1}, Radius: units.Millimeters(4), CCW: true,
			}
		},
		"clockwise": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(4)}
		},
		"infinite radius": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(math.Inf(1)), CCW: true}
		},
	} {
		require.False(t, loftcylinder.MirrorBlank(blank(edit)), name)
	}
}

func TestMirrorPairMotion(t *testing.T) {
	source := &facetedrecord.LoftMirrorJoinRecord{}
	other := &facetedrecord.LoftMirrorJoinRecord{}
	identity := r3.Identity()
	motion, ok := loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorJoin: source, Xform: identity,
	}, source)
	require.True(t, ok)
	require.Equal(t, identity, motion)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorJoin: other, Xform: identity,
	}, source)
	require.False(t, ok, "the pair joins another source")

	rotated, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/4))
	require.NoError(t, err)
	motion, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: rotated},
		Xform:          rotated,
	}, source)
	require.True(t, ok)
	require.Equal(t, rotated, motion)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: other, Motion: rotated},
		Xform:          rotated,
	}, source)
	require.False(t, ok, "the copy places another source")
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: rotated},
		Xform:          identity,
	}, source)
	require.False(t, ok, "the held mesh lost the recorded motion")
	shifted, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: shifted},
		Xform:          shifted,
	}, source)
	require.False(t, ok, "the motion is not axial")
}

// TestMirrorChamferUpperShifts pins each source tooth's upper shift to its own
// authenticated placement. The pairs and the sorted source teeth come in
// different orders, so a missing, reordered or reversed shift list fails.
func TestMirrorChamferUpperShifts(t *testing.T) {
	middle, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	identity := r3.Identity()
	// Whether a Z rotation's EZ.Z rounds to one or one step below it depends
	// on the platform's sine and cosine, so take the first whole degree of
	// each kind this platform produces instead of pinning one angle.
	rotation := func(want func(r3.Transform) bool) r3.Transform {
		for degree := 1; degree < 360; degree++ {
			motion, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(float64(degree)))
			require.NoError(t, err)
			if want(motion) {
				return motion
			}
		}
		require.FailNow(t, "no whole-degree Z rotation of the wanted kind")
		return r3.Transform{}
	}
	near := rotation(loftmesh.NearAxialPlacement)
	axial := rotation(func(motion r3.Transform) bool {
		return motion != identity && loftcylinder.AxialPlacement(motion)
	})

	identityRecord := &loftrecord.SourceIdentity{}
	join := &facetedrecord.LoftMirrorJoinRecord{Lower: &topo.Body{Payload: loftrecord.Payload{
		SourceIdentity: identityRecord, Frame1: middle, Xform: identity, AuthenticatedReconstruction: true,
	}}}
	area := measurement.Measurement{Value: units.SquareMillimeters(40), Bound: units.SquareMillimeters(1)}
	pair := func(motion r3.Transform) *topo.Body {
		fp := facetedrecord.Payload{ContactAudited: true, Xform: motion,
			LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: join, Motion: motion}}
		if motion == identity {
			fp = facetedrecord.Payload{ContactAudited: true, Xform: identity, LoftMirrorJoin: join}
		}
		return &topo.Body{Payload: fp, Area: area}
	}
	tooth := func(motion r3.Transform) *topo.Body {
		return &topo.Body{Payload: loftrecord.Payload{
			SourceIdentity: identityRecord, Xform: motion, AuthenticatedReconstruction: true,
		}}
	}
	shift, swept, ok := loftcylinder.MirrorUpperBridge(pair(near), near, 1)
	require.True(t, ok)
	// The Join accumulates each pair's charge with an upward sum.
	swept = proofbound.AbsSumUpper(0, swept)
	record := func(motions ...r3.Transform) *facetedrecord.LoftMirrorCylinderBoreRecord {
		pairs := make([]*topo.Body, len(motions))
		for i, motion := range motions {
			pairs[i] = pair(motion)
		}
		return &facetedrecord.LoftMirrorCylinderBoreRecord{Join: &facetedrecord.LoftMirrorCylinderJoinRecord{
			Source: join, Pairs: pairs, BridgeBound: shift, BridgeVolume: swept,
		}}
	}
	source := loftcylinder.ChamferSource{
		OriginalTeeth: []*topo.Body{tooth(near), tooth(axial), tooth(identity)},
	}

	shifts, err := loftcylinder.MirrorChamferUpperShifts(record(identity, axial, near), source)
	require.NoError(t, err)
	require.Len(t, shifts, 3)
	// The exact shift 2m(1-EZ.Z) with m = 1, read from the fixture's own placement.
	want := new(big.Rat).Mul(big.NewRat(2, 1),
		new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(near.Basis().EZ.Z)))
	require.Positive(t, want.Sign(), "the near-axial tooth must have a shift")
	for i, expected := range []*big.Rat{want, new(big.Rat), new(big.Rat)} {
		require.NotNil(t, shifts[i], "tooth %d", i)
		require.Zero(t, expected.Cmp(shifts[i]), "tooth %d has shift %s, want %s", i, shifts[i], expected)
	}

	shifts, err = loftcylinder.MirrorChamferUpperShifts(record(identity, axial),
		loftcylinder.ChamferSource{OriginalTeeth: []*topo.Body{tooth(axial), tooth(identity)}})
	require.NoError(t, err)
	require.Nil(t, shifts, "axial teeth need no reflected check")

	for name, bad := range map[string]func(*facetedrecord.LoftMirrorCylinderBoreRecord){
		"bridge bound": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.BridgeBound = math.Nextafter(shift, 0)
		},
		"bridge volume": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.BridgeVolume = math.Nextafter(swept, 0)
		},
		"duplicate placement": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.Pairs[1] = pair(near)
		},
	} {
		r := record(identity, axial, near)
		bad(r)
		_, err := loftcylinder.MirrorChamferUpperShifts(r, source)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
	}
	_, err = loftcylinder.MirrorChamferUpperShifts(record(identity, axial, near),
		loftcylinder.ChamferSource{OriginalTeeth: []*topo.Body{tooth(near), tooth(axial), tooth(near)}})
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a source tooth has no matching placement")
}

func TestMirrorLevels(t *testing.T) {
	t.Parallel()
	require.True(t, loftcylinder.MirrorLevels(0, 1, 2))
	require.True(t, loftcylinder.MirrorLevels(-3, -1, 1))
	require.False(t, loftcylinder.MirrorLevels(0, math.Nextafter(1, 2), 2))
	require.False(t, loftcylinder.MirrorLevels(0, 1, math.Nextafter(2, 3)))
	require.False(t, loftcylinder.MirrorLevels(0, math.NaN(), 2))
	require.False(t, loftcylinder.MirrorLevels(math.Inf(-1), 0, math.Inf(1)))
}

// TestPlaceTeeth drives the helper with placements that fail at chosen teeth:
// the lowest failing tooth's error comes back, every tooth below it ran, and a
// clean run keeps index order.
func TestPlaceTeeth(t *testing.T) {
	t.Parallel()
	for _, n := range []int{0, 1, 7} {
		for _, workers := range []int{1, 2, 4} {
			bodies := make([]*topo.Body, n)
			for i := range bodies {
				bodies[i] = &topo.Body{}
			}
			teeth, err := loftcylinder.PlaceTeeth(n, workers, func(i int) (*topo.Body, error) { return bodies[i], nil })
			require.NoError(t, err)
			require.Len(t, teeth, n)
			for i := range teeth {
				require.Same(t, bodies[i], teeth[i])
			}
		}
	}
	for _, failing := range [][]int{{0}, {2, 5}, {5, 2}, {6}, {1, 2, 3, 4, 5, 6}} {
		for _, workers := range []int{1, 2, 4} {
			ran := make([]atomic.Bool, 7)
			teeth, err := loftcylinder.PlaceTeeth(7, workers, func(i int) (*topo.Body, error) {
				ran[i].Store(true)
				if slices.Contains(failing, i) {
					return nil, fmt.Errorf("tooth %d", i)
				}
				return &topo.Body{}, nil
			})
			lowest := slices.Min(failing)
			require.EqualError(t, err, fmt.Sprintf("tooth %d", lowest), "failing %v, %d workers", failing, workers)
			require.Nil(t, teeth)
			for i := range lowest {
				require.True(t, ran[i].Load(), "tooth %d below the first failure did not run", i)
			}
		}
	}
}

func TestCheckMirrorPairs(t *testing.T) {
	t.Parallel()
	doc := &topo.Document{}
	record := &facetedrecord.LoftMirrorJoinRecord{}
	quarter, err := r3.FromBasis(r3.Basis{EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1}}, r3.Vec{})
	require.NoError(t, err)
	source := func() *topo.Body {
		return &topo.Body{Doc: doc, Payload: facetedrecord.Payload{
			ContactAudited: true, Xform: r3.Identity(), LoftMirrorJoin: record}}
	}
	turned := func() *topo.Body {
		return &topo.Body{Doc: doc, Payload: facetedrecord.Payload{ContactAudited: true, Xform: quarter,
			LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: record, Motion: quarter}}}
	}
	t.Run("every pair passes", func(t *testing.T) {
		c := loftcylinder.CheckMirrorPairs(doc, []*topo.Body{source(), turned()}, record, 1)
		require.Equal(t, 2, c.Stop)
		require.NoError(t, c.Err)
		require.Equal(t, []r3.Transform{r3.Identity(), quarter}, c.Motions)
		require.Equal(t, []float64{0, 0}, c.UpperBridges)
		require.Zero(t, c.BridgeBound)
		require.Zero(t, c.BridgeVolume)
	})
	refused := []struct {
		name string
		pair *topo.Body
	}{
		{name: "another document", pair: &topo.Body{Doc: &topo.Document{}, Payload: facetedrecord.Payload{
			ContactAudited: true, Xform: r3.Identity(), LoftMirrorJoin: record}}},
		{name: "not contact audited", pair: &topo.Body{Doc: doc, Payload: facetedrecord.Payload{
			Xform: r3.Identity(), LoftMirrorJoin: record}}},
		{name: "not faceted", pair: &topo.Body{Doc: doc, Payload: prismrecord.Payload{}}},
	}
	for _, tc := range refused {
		t.Run(tc.name, func(t *testing.T) {
			c := loftcylinder.CheckMirrorPairs(doc, []*topo.Body{source(), tc.pair, turned()}, record, 1)
			require.Equal(t, 1, c.Stop)
			require.False(t, c.Admit)
			require.NoError(t, c.Err)
			require.Equal(t, r3.Identity(), c.Motions[0])
		})
	}
	t.Run("a pair of another source", func(t *testing.T) {
		other := &topo.Body{Doc: doc, Payload: facetedrecord.Payload{ContactAudited: true, Xform: r3.Identity(),
			LoftMirrorJoin: &facetedrecord.LoftMirrorJoinRecord{}}}
		c := loftcylinder.CheckMirrorPairs(doc, []*topo.Body{source(), other}, record, 1)
		require.Equal(t, 1, c.Stop)
		require.True(t, c.Admit)
		require.ErrorIs(t, c.Err, decaderr.ErrUnsupported)
		require.ErrorContains(t, c.Err, "pair 2 has no authenticated axial placement")
	})
}
