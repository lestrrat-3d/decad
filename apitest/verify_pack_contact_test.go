package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin two readings of agent-docs/interference-design.md §3.2
// against a wound yarn pack, an annulus of outer radius 30.65 about the line
// x = −140, z = 690 along y, that a body touches from outside: the coaxial
// separator's Revolve reach, and the same-set reading of a Transform by the
// identity. No held mesh decides either contact. The numbers are a spinning
// wheel's, so each flush contact rounds as it does there.

// packRadius is the yarn pack's outer radius, 31 less the yarn's radius.
const packRadius = 31 - 0.35

// packAxis is a point of the pack's axis.
var packAxis = r3.Vec{X: -140, Z: 690}

// yarnPack extrudes the annulus 17 ≤ ρ ≤ packRadius about the pack's axis
// from y = 200 to y = 88.
func yarnPack(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	f, err := r3.NewFrame(packAxis.Add(r3.Vec{Y: 200}), r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	for _, r := range []float64{packRadius, 17} {
		c := s.CreatePoint(0, 0)
		s.Fix(c)
		s.CreateCircle(c, r)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(112), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// witnessBar revolves the section inner ≤ ρ ≤ inner + 0.2, 0 ≤ z ≤ 8 about
// the z axis, through the sweep, and places it on the pack's axis by a frame
// turned about y by an angle whose cosine and sine round: its lift is a
// rotation by rounding only, as a label bonded to a turned bobbin's pack is.
func witnessBar(t *testing.T, doc *decad.Document, inner float64, sweep decad.AngularExtent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	rect := s.CreateRectangle(inner, 0, inner+0.2, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.ConstructionAxis{Dir: r3.Vec{Z: 1}}, sweep)
	require.NoError(t, err)
	angle := math.Remainder((7-100/(2*math.Pi*31))*math.Pi/3+math.Pi/2-math.Asin(2/packRadius), 2*math.Pi)
	c, sn := math.Cos(angle), math.Sin(angle)
	f, err := r3.NewFrame(packAxis.Add(r3.Vec{Y: 148}), r3.Vec{X: c, Z: sn}, r3.Vec{X: -sn, Z: c})
	require.NoError(t, err)
	xf, err := r3.FromFrame(f)
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, xf)
	require.NoError(t, err)
	return placed
}

// TestVerifyCoaxialRevolveReach holds the Revolve reach to its sweep: a
// partial bar flush on the pack under a rotation by rounding is decided,
// because the rounding moves its swept angles off the pack. The same bar
// turned a whole revolution reaches 1e-15 mm inside the pack along another
// angle, and a bar sunk 0.001 mm into the pack overlaps it; neither is
// separated.
//
// Leg shown to fail: reading no Revolve outside a cylinder leaves the flush
// partial bar with a pair diagnostic.
func TestVerifyCoaxialRevolveReach(t *testing.T) {
	t.Parallel()
	partial := decad.AngleExtent{A: units.Radians(2 * math.Asin(2/packRadius)), Dir: decad.Along}
	for _, tc := range []struct {
		name      string
		inner     float64
		sweep     decad.AngularExtent
		separated bool
	}{
		{"partial bar flush on the pack", packRadius, partial, true},
		{"whole ring flush on the pack", packRadius, decad.FullRevolution{}, false},
		{"partial bar sunk into the pack", packRadius - 0.001, partial, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			pack := yarnPack(t, doc)
			bar := witnessBar(t, doc, tc.inner, tc.sweep)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, pack, bar)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the Revolve reach decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}

// yarnRod extrudes a Ø0.7 rod from a to c, its section plane holding the y
// axis wherever a and c share their y.
func yarnRod(t *testing.T, doc *decad.Document, a, c r3.Vec) *decad.Body {
	t.Helper()
	d := c.Sub(a)
	n, ok := d.Normalize()
	require.True(t, ok)
	ref := r3.Vec{Z: 1}
	if math.Abs(n.Z) > 0.9 {
		ref = r3.Vec{X: 1}
	}
	u, ok := ref.Cross(n).Normalize()
	require.True(t, ok)
	f, err := r3.NewFrame(a, u, n.Cross(u))
	require.NoError(t, err)
	xf, err := r3.FromFrame(f)
	require.NoError(t, err)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 0.35)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(d.Len()), Dir: decad.Along})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, xf)
	require.NoError(t, err)
	return placed
}

// yarnPoint is the flyer point (x, y, z), about the pack's axis, turned by
// the flyer's angle about it.
func yarnPoint(x, y, z float64) r3.Vec {
	angle := math.Remainder(7*math.Pi/3, 2*math.Pi)
	c, s := math.Cos(angle), math.Sin(angle)
	return r3.Vec{X: c*x - s*z, Y: y, Z: s*x + c*z}.Add(packAxis.Add(r3.Vec{Y: 278}))
}

// packTangent is the flyer point at y where the line from the flyer point
// (px, y, pz) touches the circle of radius rt about the pack's axis.
func packTangent(px, y, pz, rt float64) r3.Vec {
	p := r3.Vec{X: px, Z: pz}
	d2 := p.Dot(p)
	tangent := p.Scale(rt * rt / d2).Sub(r3.Vec{X: -p.Z, Z: p.X}.Scale(rt * math.Sqrt(d2-rt*rt) / d2))
	return yarnPoint(tangent.X, y, tangent.Z)
}

// TestVerifyIdentityPlacedChain reads a Transform by the identity as its
// source: a yarn rod whose side rests on the pack is decided by the rod's
// side plane (the slab separator), and so is the same rod as the last span
// of a refined chain placed by the identity, read through that placement, the
// Refine and the Union. A rod reaching 1e-6 mm into the pack is not
// separated.
//
// Leg shown to fail: reading no identity Transform as its source leaves the
// placed chain with a pair diagnostic.
func TestVerifyIdentityPlacedChain(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		rt        float64
		chain     bool
		separated bool
	}{
		{"rod resting on the pack", 31, false, true},
		{"refined chain placed by the identity", 31, true, true},
		{"rod into the pack", 31 - 1e-6, false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			pack := yarnPack(t, doc)
			p := yarnPoint(58, -160, 22)
			yarn := yarnRod(t, doc, p, packTangent(58, -160, 22, tc.rt))
			if tc.chain {
				feed := yarnRod(t, doc, yarnPoint(58, -224, 22), p)
				joined, err := decad.Union(t.Context(), feed, yarn)
				require.NoError(t, err)
				refined, err := decad.Refine(t.Context(), joined, units.Millimeters(0.12))
				require.NoError(t, err)
				yarn, err = decad.Transform(t.Context(), refined, r3.Identity())
				require.NoError(t, err)
			}
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, pack, yarn)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the history walk decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}
