package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/tessellation-design.md §13's increment T4
// (docs/tessellation-reach-design.md §6, R5): the revolve as a MESH BOOLEAN
// OPERAND, admitted by §11's occupied-volume proof.
//
// Every fixture sweeps a QUARTER or a HALF turn rather than a whole one. The
// geometry a whole turn would add is already covered by the export tests beside
// this file, and the angular count a boolean's own internal tolerance asks for
// makes a full revolve's facet-pair audit the most expensive thing in the suite
// — so the sweep is cut where it costs nothing the proof needs.

// quarterCylinder is the radius-8, 10 mm cylinder swept through 90° from the
// sketch plane: the section is solidSketch's rectangle with one edge ON the
// axis, so the sweep is solid and both partial caps are exact rectangles.
func quarterCylinder(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := solidSketch(t)
	b, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// revolveBoxBody extrudes the rectangle u∈[u0,u1], v∈[v0,v1] of the XY plane by
// h along +Z: an all-planar prism, so it contributes no chord error of its own
// and every bound the pair composes is the revolve's.
func revolveBoxBody(t *testing.T, doc *decad.Document, u0, v0, u1, v1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// mm3 reads a volume measurement and its bound in cubic millimetres.
func mm3(t *testing.T, b *decad.Body) (float64, float64) {
	t.Helper()
	m, err := b.Volume()
	require.NoError(t, err)
	v, err := m.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	d, err := m.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v, d
}

func TestRevolveUnionWithAPrismMeasuresTheAnalyticVolume(t *testing.T) {
	doc := decad.New()
	cyl := quarterCylinder(t, doc)
	// x∈[2,6], y∈[6,12], z∈[1,5]: the box crosses the cylinder's curved wall,
	// so the union's own volume is the closed form below and the result is a
	// genuine crossing rather than a containment certificate.
	box := translated(t, revolveBoxBody(t, doc, 2, 6, 6, 12, 4), 0, 0, 1)

	got, err := decad.Union(cyl, box)
	require.NoError(t, err)
	volume, bound := mm3(t, got)
	require.Positive(t, bound)
	require.False(t, math.IsInf(bound, 1))

	// The quarter cylinder, plus the box, less the part of the box already
	// inside the cylinder: for z from 1 to 5 the wall sits at y = √(64 − z²),
	// which stays above the box's own y = 6 throughout, so the shared
	// cross-section is ∫(√(64 − z²) − 6) dz over that span.
	quarter := math.Pi * 64 * 10 / 4
	area := func(z float64) float64 { return z/2*math.Sqrt(64-z*z) + 32*math.Asin(z/8) }
	shared := 4 * (area(5) - area(1) - 6*4)
	want := quarter + 4*6*4 - shared
	require.InDelta(t, want, volume, bound,
		`the published volume bound must cover the gap to the analytic union`)

	mesh, err := got.Tessellate(units.Millimeters(0.5))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}

func TestRevolveCutByAPrismMeasuresTheAnalyticVolume(t *testing.T) {
	doc := decad.New()
	cyl := quarterCylinder(t, doc)
	// The same crossing box, moved down in y so it takes a bite out of the
	// cylinder rather than adding to it.
	box := translated(t, revolveBoxBody(t, doc, 2, 2, 6, 12, 4), 0, 0, 1)

	got, err := decad.Cut(cyl, box)
	require.NoError(t, err)
	volume, bound := mm3(t, got)
	require.Positive(t, bound)

	// The removed part is the box's own cross-section between y = 2 and the
	// wall, over x∈[2,6] and z∈[1,5].
	quarter := math.Pi * 64 * 10 / 4
	area := func(z float64) float64 { return z/2*math.Sqrt(64-z*z) + 32*math.Asin(z/8) }
	removed := 4 * (area(5) - area(1) - 2*4)
	require.InDelta(t, quarter-removed, volume, bound)
}

func TestRevolveUnionWithAnotherRevolveObeysInclusionExclusion(t *testing.T) {
	// Two quarter cylinders, the second moved clear of every plane the first's
	// own caps lie in, so no face pair is coplanar. Their union and their
	// intersection are computed by two INDEPENDENT boolean runs, and
	// |A ∪ B| + |A ∩ B| = |A| + |B| ties the two answers to the operands'
	// analytic volumes without either result standing in for the other.
	quarter := math.Pi * 64 * 10 / 4

	unionDoc := decad.New()
	ua := quarterCylinder(t, unionDoc)
	ub := translated(t, quarterCylinder(t, unionDoc), 4, 4, 4)
	joined, err := decad.Union(ua, ub)
	require.NoError(t, err)
	unionVol, unionBound := mm3(t, joined)

	meetDoc := decad.New()
	ma := quarterCylinder(t, meetDoc)
	mb := translated(t, quarterCylinder(t, meetDoc), 4, 4, 4)
	shared, err := decad.Intersect(ma, mb)
	require.NoError(t, err)
	meetVol, meetBound := mm3(t, shared)

	require.Positive(t, meetVol)
	require.Less(t, meetVol, unionVol)
	require.Greater(t, unionVol, quarter)
	require.Less(t, unionVol, 2*quarter)
	require.InDelta(t, 2*quarter, unionVol+meetVol, unionBound+meetBound,
		`the two operands' proven bounds must cover the inclusion-exclusion residue`)

	mesh, err := joined.Tessellate(units.Millimeters(0.5))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}

func TestRevolveBooleanRefusesAHiddenTangency(t *testing.T) {
	// A half turn puts the cylinder's own tangent line at z = 8 well inside the
	// swept wall. The box's floor sits exactly on it, so the TRUE surfaces touch
	// along a line while the held facets — inscribed in the wall they chord —
	// merely come within the pair's chord tolerance of it. §11 step 4 refuses
	// that question rather than letting the chord placement answer it.
	doc := decad.New()
	s, p := solidSketch(t)
	cyl, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along})
	require.NoError(t, err)
	box := translated(t, revolveBoxBody(t, doc, 2, -4, 6, 4, 6), 0, 0, 8)

	_, err = decad.Union(cyl, box)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code,
		`a tangency the chords cannot decide is a contact refusal, not a staging one`)
}

func TestRevolveBooleanRefusesAShallowCrossing(t *testing.T) {
	// The quarter turn's start cap is an exact planar rectangle in the z = 0
	// half plane, so the pair meets there with almost no face bound of its own
	// and the crossing is decided. What is NOT decided is the RIM the boolean
	// mints along it: a rim vertex sits where the two chord planes cross, and
	// the region the true curve may occupy is a tube of half-width
	// (δA + δB)/sin θ about it. Tip the box until that tube reaches the pair's
	// own diameter and the answer stops meaning anything, so the call refuses.
	tipped := func(t *testing.T, alpha float64) error {
		t.Helper()
		doc := decad.New()
		cyl := quarterCylinder(t, doc)
		box := revolveBoxBody(t, doc, 1, 1, 7, 6, 4)
		spun, err := r3.RotationAround(r3.Vec{X: 4, Y: 3.5}, r3.Vec{X: 1}, units.Radians(alpha))
		require.NoError(t, err)
		moved, err := box.Placed(spun)
		require.NoError(t, err)
		_, err = decad.Union(cyl, moved)
		return err
	}

	err := tipped(t, 1e-5)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code)

	// The same crossing at a hundred times the angle is answered, so the
	// refusal above is the amplification and not the fixture.
	require.NoError(t, tipped(t, 1e-3))
}

func TestRevolveBooleanChargesBothOperandVolumeProofs(t *testing.T) {
	// The result's volume bound is the sum of the two operands' own
	// occupied-volume proofs plus the final weld (§11 step 6). An ALL-PLANAR
	// prism proves zero, so replacing the prism operand with a second revolve
	// must raise the bound rather than leave it alone.
	withPrism := decad.New()
	_, boundA := func() (float64, float64) {
		cyl := quarterCylinder(t, withPrism)
		box := translated(t, revolveBoxBody(t, withPrism, 2, 6, 6, 12, 4), 0, 0, 1)
		got, err := decad.Union(cyl, box)
		require.NoError(t, err)
		return mm3(t, got)
	}()

	withRevolve := decad.New()
	_, boundB := func() (float64, float64) {
		a := quarterCylinder(t, withRevolve)
		b := translated(t, quarterCylinder(t, withRevolve), 4, 4, 4)
		got, err := decad.Union(a, b)
		require.NoError(t, err)
		return mm3(t, got)
	}()

	require.Positive(t, boundA)
	require.Greater(t, boundB, boundA,
		`two revolve operands charge two occupied-volume proofs, an all-planar prism only one`)
}
