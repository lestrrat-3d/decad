package apitest_test

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The stepped-shaft fixtures of agent-docs/general-boolean-design.md §3
// "Revolved shaft as a prism": a full Revolve of a staircase with one side on
// the axis is read as its stack of coaxial circular prisms, so a
// perpendicular box spanning its length cuts it through class B's crossing
// reach. Each tool is drawn on an offset datum plane, so it shares the
// shaft's placement.
//
// Shown-to-fail: without the stepped arm in booleandispatch.classB each exact
// fixture takes the mesh path and its volume bound is far above 1e-6 mm³.
// Without the box-apart reading of a face the tool does not reach, or the
// shoulder whose hole is apart from the tool, the 10/6 and 6/10 fixtures take
// the mesh path. Without the one-kept-face check the slot splitting the wide
// step builds through class B. Without the step-contiguity check of the
// stepped slice, the 10/6/9 shaft's flat builds a loop that skips the middle
// step and the Cut refuses; without the miss on a tool wall across the axis
// strictly inside the span, the keyseat ending inside the shaft panics.

// stepRevolve revolves the staircase whose steps have the given radii, each
// stepLen long along x from x = 0, about the x axis.
func stepRevolve(t *testing.T, sc *retentionScene, radii ...float64) *decad.Body {
	t.Helper()
	const stepLen = 20.0
	end := stepLen * float64(len(radii))
	pts := [][2]float64{{0, 0}, {end, 0}}
	for i, r := range slices.Backward(radii) {
		pts = append(pts, [2]float64{stepLen * float64(i+1), r}, [2]float64{stepLen * float64(i), r})
	}
	s, profile := polygonSketch(t, pts)
	shaft, err := decad.Revolve(t.Context(), sc.doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return shaft
}

// discLess is the area of the disc of radius r less removed, at the closed
// forms' working precision.
func discLess(r float64, removed *big.Float) *big.Float {
	disc := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(r*r))
	return disc.Sub(disc, removed)
}

// chordSegment is the area of the disc of radius r above the chord at height
// h = flatZ: r²·acos(h/r) − h·√(r² − h²), with acos(h/r) = π/2 −
// atan(h/√(r² − h²)).
func chordSegment(r float64) *big.Float {
	h := flatZ
	root := shaftSqrt(r*r - h*h)
	half := new(big.Float).SetPrec(junctionPrec).Quo(junctionPi(), jf(2))
	acos := half.Sub(half, junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(h), root)))
	area := acos.Mul(acos, jf(r*r))
	return area.Sub(area, new(big.Float).SetPrec(junctionPrec).Mul(jf(h), root))
}

// keySeatOf is the area of the disc of radius r with |y| ≤ w above z = h:
// w·√(r² − w²) + r²·asin(w/r) − 2·w·h, with asin(w/r) = atan(w/√(r² − w²)).
func keySeatOf(r, w, h float64) *big.Float {
	root := shaftSqrt(r*r - w*w)
	asin := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(w), root))
	area := asin.Mul(asin, jf(r*r))
	area.Add(area, new(big.Float).SetPrec(junctionPrec).Mul(jf(w), root))
	return area.Sub(area, jf(2*w*h))
}

// stepsVolume sums each 20 mm step's cut section times its length.
func stepsVolume(sections ...*big.Float) *big.Float {
	sum := new(big.Float).SetPrec(junctionPrec)
	for _, s := range sections {
		sum.Add(sum, s)
	}
	return sum.Mul(sum, jf(20))
}

// requireClassBStepped checks a cut stepped shaft built through class B: no
// Faceted face, the given cylinder and plane counts, a volume within its
// bound of truth, and a mesh whose boundary and occupied volume verify.
func requireClassBStepped(t *testing.T, got *decad.Body, cylinders, planes int, truth *big.Float) {
	t.Helper()
	gotCylinders, gotPlanes := 0, 0
	for _, f := range got.Faces() {
		switch f.Surface().(type) {
		case decad.Faceted:
			require.Fail(t, "a class-B result has no Faceted face")
		case decad.Cylinder:
			gotCylinders++
		case decad.Plane:
			gotPlanes++
		}
	}
	require.Equal(t, cylinders, gotCylinders)
	require.Equal(t, planes, gotPlanes)
	volume, err := got.Volume()
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, bound, 1e-6, `class B, not the mesh path, measures the cut`)
	requireJunctionEncloses(t, `cut stepped shaft volume`, value, bound, truth)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSteppedShaftClassBCut(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name              string
		radii             []float64
		v0, v1            float64
		cylinders, planes int
		sections          func() []*big.Float
	}{
		{
			// The flat at z = 7 crosses both steps: the shoulder keeps a
			// C-shaped face and the flat is one stepped face.
			name: "D-flat across both steps", radii: []float64{10, 8}, v0: -20, v1: 20, cylinders: 2, planes: 4,
			sections: func() []*big.Float {
				return []*big.Float{discLess(10, chordSegment(10)), discLess(8, chordSegment(8))}
			},
		},
		{
			// The r = 6 step lies below the flat: its wall stays whole and
			// the shoulder keeps it as a hole.
			name: "D-flat on the wide step only", radii: []float64{10, 6}, v0: -20, v1: 20, cylinders: 2, planes: 4,
			sections: func() []*big.Float {
				return []*big.Float{discLess(10, chordSegment(10)), discLess(6, big.NewFloat(0))}
			},
		},
		{
			// A shaft stepping up along x: its shoulder faces -x.
			name: "D-flat on a step up", radii: []float64{6, 10}, v0: -20, v1: 20, cylinders: 2, planes: 4,
			sections: func() []*big.Float {
				return []*big.Float{discLess(6, big.NewFloat(0)), discLess(10, chordSegment(10))}
			},
		},
		{
			name: "keyway across both steps", radii: []float64{10, 8}, v0: -3, v1: 3, cylinders: 2, planes: 6,
			sections: func() []*big.Float {
				return []*big.Float{discLess(10, keySeatOf(10, 3, 7)), discLess(8, keySeatOf(8, 3, 7))}
			},
		},
		{
			name: "D-flat across three steps", radii: []float64{8, 10, 9}, v0: -20, v1: 20, cylinders: 3, planes: 5,
			sections: func() []*big.Float {
				return []*big.Float{
					discLess(8, chordSegment(8)), discLess(10, chordSegment(10)), discLess(9, chordSegment(9)),
				}
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := newRetentionScene()
			shaft := stepRevolve(t, sc, tc.radii...)
			end := 20 * float64(len(tc.radii))
			got, err := decad.Cut(t.Context(), shaft, shaftTool(t, sc, flatPlane(t, sc), -5, tc.v0, end+5, tc.v1))
			require.NoError(t, err)
			requireClassBStepped(t, got, tc.cylinders, tc.planes, stepsVolume(tc.sections()...))
		})
	}
}

func TestSteppedShaftClassBBlendsRefuse(t *testing.T) {
	t.Parallel()

	// The stepped flat carries the crossing's section displacement, as a
	// plain shaft's does, so brep-modify's SB1 refuses a blend of its edges:
	// the stepped flat reads as no prism (brep-modify §4.4).
	sc := newRetentionScene()
	cut, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 8),
		shaftTool(t, sc, flatPlane(t, sc), -5, -20, 45, 20))
	require.NoError(t, err)
	var edge *decad.EdgeQuery
	for _, e := range cut.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		a, c := e.Start().Position().Value, e.End().Position().Value
		if a.Z == flatZ && c.Z == flatZ && a.X != c.X {
			edge = decad.Edges(decad.EndpointAt(a), decad.EndpointAt(c))
			break
		}
	}
	require.NotNil(t, edge, `the flat has a long edge`)
	_, err = decad.Fillet(t.Context(), cut, edge.Exactly(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, `brep-modify SB1`)
	_, err = decad.Chamfer(t.Context(), cut, edge.Exactly(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, `brep-modify SB1`)
}

func TestSteppedShaftClassBMisses(t *testing.T) {
	t.Parallel()

	t.Run("flat meeting two steps apart", func(t *testing.T) {
		t.Parallel()
		// The flat at z = 7 meets the r = 10 and r = 9 steps but not the
		// r = 6 step between them, so its slice of the shaft is two loops.
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 6, 9),
			shaftTool(t, sc, flatPlane(t, sc), -5, -20, 65, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("slot splitting the wide step", func(t *testing.T) {
		t.Parallel()
		// A 1 mm slab from z = 7 cuts the r = 10 step in two pieces, and
		// the shoulder's r = 6 hole lies in one of them, which the build
		// does not place.
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(flatPlane(t, sc))
		require.NoError(t, err)
		r := s.CreateRectangle(-5, -20, 45, 20)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		slot, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 6), slot)
		requireShaftMeshPath(t, got, err)
	})

	t.Run("slot beside the narrow step", func(t *testing.T) {
		t.Parallel()
		// The slot y in [4, 6], z >= 5 is apart from the r = 6 step
		// (y² + z² >= 41 there) but enters the box of the shoulder's r = 6
		// hole, so the shoulder's scene holds the hole and misses. The mesh
		// path's volume holds 20·(100π + 36π − A), with A the r = 10 disc's
		// area in the slot: ∫(√(100 − y²) − 5) dy over [4, 6] =
		// (48 + 100·asin(0.6))/2 − (4·√84 + 100·asin(0.4))/2 − 10, with
		// asin(0.6) = atan(6/8) and asin(0.4) = atan(4/√84).
		sc := newRetentionScene()
		plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), 5)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 6), shaftTool(t, sc, plane, -5, 4, 45, 6))
		requireShaftMeshPath(t, got, err)

		at := func(y float64, root, asin *big.Float) *big.Float {
			v := new(big.Float).SetPrec(junctionPrec).Mul(jf(y), root)
			return v.Add(v, new(big.Float).SetPrec(junctionPrec).Mul(jf(100), asin))
		}
		hi := at(6, jf(8), junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(6), jf(8))))
		s84 := shaftSqrt(84)
		lo := at(4, s84, junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(4), s84)))
		area := hi.Sub(hi, lo)
		area.Quo(area, jf(2))
		area.Sub(area, jf(10))
		truth := stepsVolume(discLess(10, area), discLess(6, big.NewFloat(0)))
		volume, err := got.Volume()
		require.NoError(t, err)
		value, err := volume.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		bound, err := volume.Bound.In(units.CubicMillimeter)
		require.NoError(t, err)
		requireJunctionEncloses(t, `slotted stepped shaft volume`, value, bound, truth)
	})

	t.Run("flat tangent to the narrow step", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), 8)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 8), shaftTool(t, sc, plane, -5, -20, 45, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("keyseat ending inside the shaft", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 8),
			shaftTool(t, sc, flatPlane(t, sc), 15, -3, 25, 3))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("tool wall in the end plane", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 8),
			shaftTool(t, sc, flatPlane(t, sc), 0, -20, 45, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("Transform-placed tool", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), stepRevolve(t, sc, 10, 8), flatTool(t, sc))
		requireShaftMeshPath(t, got, err)
	})
}
