package decad

import (
	"fmt"
	"math"
	"math/cmplx"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests pin Centroid's shift form (docs/loft-gear-bounds-design.md §3):
// the bound is rounding + epsV·R_c/clearance, with R_c measured from the
// published centroid rather than from the mass anchor.

// The lobe section is two circular arcs: a radius-5 arc about the sketch
// origin from 20° counter-clockwise to 330°, closed on the right by a bulge
// arc whose centre sits lobeBulgeOffset along the chord's outward normal from
// the chord's midpoint. The bulge pulls the centroid right of the origin, so
// the true boundary point farthest from the centroid lies on the big arc near
// 180°, between two stations rather than on one. That is the point a
// held-vertex radius alone misses, and what R_c's max(matchedDelta, delta)
// term covers. Every wall cell is chorded, so the loft's denoted wall is the
// ruled surface everywhere and its exact centroid has a closed form.
const (
	lobeRadius     = 5.0
	lobeArcStart   = 20 * math.Pi / 180
	lobeArcEnd     = 330 * math.Pi / 180
	lobeHeight     = 10.0
	lobeTwistAngle = 20 * math.Pi / 180
)

// lobeArc is one circular arc of the lobe, counter-clockwise from a to b.
type lobeArc struct {
	center [2]float64
	radius float64
	a, b   float64
}

func (c lobeArc) at(alpha float64) [2]float64 {
	return [2]float64{c.center[0] + c.radius*math.Cos(alpha), c.center[1] + c.radius*math.Sin(alpha)}
}

// lobeArcs returns the big arc and the bulge arc, in boundary order.
func lobeArcs(bulgeOffset float64) [2]lobeArc {
	big := lobeArc{radius: lobeRadius, a: lobeArcStart, b: lobeArcEnd}
	p1, p2 := big.at(lobeArcStart), big.at(lobeArcEnd)
	mid := [2]float64{(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2}
	chord := [2]float64{p1[0] - p2[0], p1[1] - p2[1]}
	l := math.Hypot(chord[0], chord[1])
	normal := [2]float64{chord[1] / l, -chord[0] / l} // right of the upward chord
	center := [2]float64{mid[0] + bulgeOffset*normal[0], mid[1] + bulgeOffset*normal[1]}
	a := math.Atan2(p2[1]-center[1], p2[0]-center[0])
	b := math.Atan2(p1[1]-center[1], p1[0]-center[0])
	for b <= a {
		b += 2 * math.Pi
	}
	bulge := lobeArc{center: center, radius: math.Hypot(p2[0]-center[0], p2[1]-center[1]), a: a, b: b}
	return [2]lobeArc{big, bulge}
}

// lobeSketch draws the lobe on plane: the big arc from p1 to p2, then the
// bulge arc from p2 back to p1.
func lobeSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, bulgeOffset float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(p [2]float64) *sketch.Point {
		pt := s.CreatePoint(p[0], p[1])
		s.Fix(pt)
		return pt
	}
	arcs := lobeArcs(bulgeOffset)
	p1, p2 := fixed(arcs[0].at(arcs[0].a)), fixed(arcs[0].at(arcs[0].b))
	s.CreateArc(fixed(arcs[0].center), p1, p2)
	s.CreateArc(fixed(arcs[1].center), p2, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// lobeLoft lofts the lobe from z = 0 to a copy at lobeHeight rotated by twist
// about the z axis through the sketch origin.
func lobeLoft(t *testing.T, bulgeOffset, twist float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	f0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	f1, err := r3.NewFrame(r3.NewVec(0, 0, lobeHeight),
		r3.NewVec(math.Cos(twist), math.Sin(twist), 0), r3.NewVec(-math.Sin(twist), math.Cos(twist), 0))
	require.NoError(t, err)
	pl0, err := w.CreatePlaneFromFrame(f0)
	require.NoError(t, err)
	pl1, err := w.CreatePlaneFromFrame(f1)
	require.NoError(t, err)
	s0, p0 := lobeSketch(t, w, pl0, bulgeOffset)
	s1, p1 := lobeSketch(t, w, pl1, bulgeOffset)
	body, err := New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	return body
}

// lobeTrueCentroid is the exact centroid of the solid the twisted lobe loft
// denotes. Each section point p on z = 0 is ruled to R_twist·p at lobeHeight,
// so the section at height fraction t is the lobe scaled and rotated by the
// complex factor mu(t) = (1 − t) + t·e^{i·twist}: area A·|mu|² and centroid
// mu·c, with A and c the lobe's own area and centroid. |mu|² is symmetric
// about t = 1/2, so the solid's centroid is c·(1 + e^{i·twist})/2 at half
// height. A and c come from Green's theorem over the two arcs, integrated by
// composite Simpson's rule far below the bounds this test compares against.
func lobeTrueCentroid(bulgeOffset, twist float64) r3.Vec {
	const steps = 1 << 12
	var twiceArea, x2dy, y2dx float64
	for _, arc := range lobeArcs(bulgeOffset) {
		h := (arc.b - arc.a) / steps
		for i := range steps + 1 {
			w := 2.0
			switch {
			case i == 0 || i == steps:
				w = 1
			case i%2 == 1:
				w = 4
			}
			alpha := arc.a + h*float64(i)
			p := arc.at(alpha)
			dx, dy := -arc.radius*math.Sin(alpha), arc.radius*math.Cos(alpha)
			twiceArea += w * h / 3 * (p[0]*dy - p[1]*dx)
			x2dy += w * h / 3 * p[0] * p[0] * dy
			y2dx += w * h / 3 * p[1] * p[1] * dx
		}
	}
	area := twiceArea / 2
	c := complex((x2dy/2)/area, (-y2dx/2)/area)
	xy := c * (1 + cmplx.Exp(complex(0, twist))) / 2
	return r3.NewVec(real(xy), imag(xy), lobeHeight/2)
}

// lobeTrueBoundary samples both recorded sections of the twisted lobe in
// world coordinates. Along every ruling the distance to a fixed point is
// convex, so its largest value over the true lateral surface and caps is
// attained on these two boundary curves.
func lobeTrueBoundary(bulgeOffset, twist float64, perArc int) []r3.Vec {
	var out []r3.Vec
	for _, arc := range lobeArcs(bulgeOffset) {
		for i := range perArc + 1 {
			p := arc.at(arc.a + (arc.b-arc.a)*float64(i)/float64(perArc))
			out = append(out, r3.NewVec(p[0], p[1], 0), r3.NewVec(
				p[0]*math.Cos(twist)-p[1]*math.Sin(twist),
				p[0]*math.Sin(twist)+p[1]*math.Cos(twist),
				lobeHeight,
			))
		}
	}
	return out
}

func loftCentroidRebuild(t *testing.T, pl loftPayload) (*loftMassAccumulator, loftAssembly, float64) {
	t.Helper()
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, pl.recordArea, work0, work1)
	require.NoError(t, err)
	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftmesh.PairRecords(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)
	require.True(t, sectionDelta > 0 || sectionMatchedDelta > 0, "the fixture must be chorded")
	a, err := assembleLoft(t.Context(), pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	require.NoError(t, err)
	mass := buildLoftMass(pl, a, pairs, sectionDelta, sectionMatchedDelta)
	matchedDelta := loftmesh.ChordCellDeltaUpper(sectionMatchedDelta, a.delta)
	return mass, a, matchedDelta
}

// retiredAnchorCentroidBound is the anchor-based centroid bound the shift
// form replaced, rebuilt from the proofbound helpers it composed: per
// coordinate (epsM + |c_i − anchor_i|·epsV)/clearance, with epsM read at the
// body's inf-norm extent from the mass anchor. Its epsV is the volume legs
// Volume spends today (docs/loft-gear-bounds-design.md §2), not the cap and
// seam legs it once read, so the comparison isolates the anchor-versus-shift
// form. It omits the coordinate
// rounding, so it is a LOWER bound on what that form published, which makes
// "the shift form is smaller" the stronger claim.
func retiredAnchorCentroidBound(t *testing.T, mass *loftMassAccumulator, a loftAssembly, anchor r3.Vec, matchedDelta float64, centroid r3.Vec) float64 {
	t.Helper()
	coordUpper := 0.0
	for _, tri := range a.tris {
		for _, idx := range tri {
			d := a.verts[idx].Sub(anchor)
			coordUpper = max(coordUpper, math.Abs(d.X), math.Abs(d.Y), math.Abs(d.Z))
		}
	}
	epsV, epsM := 0.0, 0.0
	if a.delta > 0 {
		areaUpper := proofbound.PerturbedAreaUpper(a.verts, a.tris, a.delta)
		epsV = proofbound.SweptVolumeAllow(a.delta, areaUpper)
		epsM = proofbound.SweptMomentAllow(a.delta, areaUpper, coordUpper+a.delta)
	}
	c := mass.Chorded
	epsV = proofbound.AbsSumUpper(epsV, c.WallLeg, c.SkirtLeg)
	epsM = proofbound.AbsSumUpper(epsM, proofbound.ChordedBoundaryMomentResidualAllow(
		matchedDelta, c.WallAreaUpper, 0, 0, c.MaxTwistOffsetUpper, coordUpper))
	vol := mass.volume(a.verts, a.tris)
	clearance := math.Nextafter(math.Abs(vol.Value.Base())-epsV, math.Inf(-1))
	require.Positive(t, clearance, "the retired form must have had a clearance to compare against")
	perCoord := 0.0
	for _, rel := range []float64{centroid.X - anchor.X, centroid.Y - anchor.Y, centroid.Z - anchor.Z} {
		perCoord = math.Max(perCoord, (epsM+math.Abs(rel)*epsV)/clearance)
	}
	return proofbound.Radius3D(perCoord)
}

// TestLoftCentroidShiftFormEnclosesTwoArcLobe builds the two-arc lobe loft
// at three bulge offsets (so three station counts), untwisted and twisted by
// 20°, plus one placed copy, and asserts against the exact centroid of the
// solid each denotes:
//
//   - the exact centroid lies within the published Bound of the published
//     centroid;
//   - R_c (CentroidRadius) reaches every densely sampled point of the true
//     boundary, which lies up to matchedDelta outside the held vertex hull;
//   - the published Bound is below the retired anchor-based form on every row.
//
// The fixture is a lobe and not a ring: a ring's chorded body has exactly the
// true solid's centroid by symmetry, so it cannot show the shift leg red. It
// has no straight segment either: a twisted LineSeg cell denotes its held
// triangle pair, not a ruled patch, which the closed form does not model.
//
// Shown to fail: dropping the shift (Bound = rounding only) fails the
// enclosure on all seven rows; deleting R_c's max(matchedDelta, delta) term
// fails the radius assertion on the six rows whose farthest true point falls
// between two stations, at the chord target shipped with this test.
func TestLoftCentroidShiftFormEnclosesTwoArcLobe(t *testing.T) {
	t.Parallel()
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(31))
	require.NoError(t, err)
	type row struct {
		name               string
		bulgeOffset, twist float64
		placed             bool
	}
	var rows []row
	for _, offset := range []float64{-3, 0, 2} {
		rows = append(rows,
			row{name: fmt.Sprintf("bulge%+g", offset), bulgeOffset: offset},
			row{name: fmt.Sprintf("bulge%+g-twisted", offset), bulgeOffset: offset, twist: lobeTwistAngle},
		)
	}
	rows = append(rows, row{name: "bulge+2-twisted-placed", bulgeOffset: 2, twist: lobeTwistAngle, placed: true})
	for _, row := range rows {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			body := lobeLoft(t, row.bulgeOffset, row.twist)
			want := lobeTrueCentroid(row.bulgeOffset, row.twist)
			boundary := lobeTrueBoundary(row.bulgeOffset, row.twist, 1<<14)
			if row.placed {
				body, err = body.Placed(t.Context(), rot)
				require.NoError(t, err)
				want = rot.Apply(want)
				for i, p := range boundary {
					boundary[i] = rot.Apply(p)
				}
			}
			cen, err := body.Centroid()
			require.NoError(t, err)
			lp := body.payload.(loftPayload)
			gap := cen.Value.Sub(want).Len()
			t.Logf("walls=%d delta=%.3g sectionDelta=%.3g bound=%.4g gap=%.4g got=%v want=%v", lp.walls, lp.delta, lp.sectionDelta, cen.Bound.Base(), gap, cen.Value, want)
			require.Positive(t, gap, "the chorded centroid must differ from the true one, or the row shows nothing")
			require.LessOrEqual(t, gap, cen.Bound.Base(), "the exact centroid must lie inside the published bound")

			mass, a, matchedDelta := loftCentroidRebuild(t, lp)
			rebuilt, err := mass.centroid(a.verts, a.tris)
			require.NoError(t, err)
			require.Equal(t, cen, rebuilt, "the rebuild must reproduce the published centroid")

			radius := mass.CentroidRadius(a.verts, a.tris, cen.Value, 0)
			farthest := 0.0
			for _, p := range boundary {
				farthest = math.Max(farthest, p.Sub(cen.Value).Len())
			}
			t.Logf("R_c=%.10g farthest true boundary point=%.10g matchedDelta=%.3g", radius, farthest, matchedDelta)
			require.GreaterOrEqual(t, radius, farthest, "R_c must reach every point of the true boundary")

			retired := retiredAnchorCentroidBound(t, mass, a, lp.xform.Apply(lp.plane0.Origin), matchedDelta, cen.Value)
			t.Logf("shift form %.4g, retired anchor form at least %.4g", cen.Bound.Base(), retired)
			require.Less(t, cen.Bound.Base(), retired, "the shift form must be below the retired anchor-based form")
		})
	}
}

// involuteGear is one helical spur gear section: module, tooth count and
// pressure angle, with each involute flank a fit spline through fitPoints
// points from the base circle to the addendum circle.
type involuteGear struct {
	module, teeth, pressure float64
	fitPoints               int
}

func (g involuteGear) radii() (float64, float64, float64) {
	r := g.module * g.teeth / 2
	return r * math.Cos(g.pressure), r - 1.25*g.module, r + g.module
}

// flank returns the right flank's points as (radius, angle) pairs.
func (g involuteGear) flank() [][2]float64 {
	rb, _, ra := g.radii()
	thetaB := math.Pi/(2*g.teeth) + (math.Tan(g.pressure) - g.pressure)
	tmax := math.Sqrt((ra/rb)*(ra/rb) - 1)
	out := make([][2]float64, g.fitPoints)
	for k := range out {
		tt := tmax * float64(k) / float64(g.fitPoints-1)
		out[k] = [2]float64{rb * math.Sqrt(1+tt*tt), -(thetaB - (tt - math.Atan(tt)))}
	}
	return out
}

// toothSketch draws ONE tooth on plane: a root line, the right flank, the tip
// arc about the gear centre, the left flank, a root line, and the line
// closing the two root points.
func (g involuteGear) toothSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	polar := func(r, ang float64) *sketch.Point {
		p := s.CreatePoint(r*math.Cos(ang), r*math.Sin(ang))
		s.Fix(p)
		return p
	}
	_, rf, _ := g.radii()
	fl := g.flank()
	thetaB := -fl[0][1]
	origin := polar(0, 0)
	rootR := polar(rf, -thetaB)
	right := make([]*sketch.Point, len(fl))
	left := make([]*sketch.Point, len(fl))
	for i, p := range fl {
		right[i] = polar(p[0], p[1])
		left[len(fl)-1-i] = polar(p[0], -p[1])
	}
	rootL := polar(rf, thetaB)
	s.CreateLine(rootR, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(origin, right[len(right)-1], left[0])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[len(left)-1], rootL)
	s.CreateLine(rootL, rootR)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// helicalToothLoft lofts one tooth 10 mm with a 20° helix: the top section is
// the bottom one rotated about the gear axis by 10·tan(20°)/pitchRadius.
func helicalToothLoft(t *testing.T, g involuteGear) *Body {
	t.Helper()
	const height = 10.0
	twist := height * math.Tan(20*math.Pi/180) / (g.module * g.teeth / 2)
	w := sketch.NewWorld()
	f0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	f1, err := r3.NewFrame(r3.NewVec(0, 0, height),
		r3.NewVec(math.Cos(twist), math.Sin(twist), 0), r3.NewVec(-math.Sin(twist), math.Cos(twist), 0))
	require.NoError(t, err)
	pl0, err := w.CreatePlaneFromFrame(f0)
	require.NoError(t, err)
	pl1, err := w.CreatePlaneFromFrame(f1)
	require.NoError(t, err)
	s0, p0 := g.toothSketch(t, w, pl0)
	s1, p1 := g.toothSketch(t, w, pl1)
	body, err := New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	return body
}

// loftCentroidRatio is the Centroid row of loftBodyBindingRatio: the bound
// over Verify's own diameter reference at the default tolerance. It also
// returns that diameter.
func loftCentroidRatio(t *testing.T, body *Body, bound float64) (float64, float64) {
	t.Helper()
	area, err := body.Area()
	require.NoError(t, err)
	in := &bodyToleranceInputs{ctx: t.Context(), body: body, area: area}
	diameter, ok := in.diameterReference()
	require.True(t, ok, "the tooth must form a diameter reference")
	_, ref, have := boundedToleranceRef(bound, toleranceRel, in.diameterReference)
	require.True(t, have, "the tooth must form a tolerance reference")
	return bound / ref, diameter
}

// TestLoftCentroidToothBoundReadsToothSize lofts ONE helical tooth of a
// module-2, 20°-pressure-angle gear at 8, 20 and 40 teeth. The mass anchor
// is the gear centre, up to 41 mm from a tooth a few millimetres across, so
// the retired anchor-based centroid bound grew with the gear. The shift form
// measures R_c from the published centroid instead, so it reads the tooth.
//
// Each row asserts that R_c is within the tooth's own diameter and that the
// Centroid reading's ratio is below both the retired form's and 2.5e-4·4.
// Verify's verdict is not asserted: Volume's binding residual is §2's
// increment, not this one.
//
// Shown to fail: the retired anchor-based form, rebuilt beside it over the
// volume legs Volume spends today, exceeds 2.5e-4·4 on the z40 row, where the
// anchor is farthest from the tooth, as the assertion on it records.
func TestLoftCentroidToothBoundReadsToothSize(t *testing.T) {
	t.Parallel()
	const threshold = 2.5e-4 * 4
	for _, row := range []struct {
		teeth        float64
		retiredFails bool
	}{{8, false}, {20, false}, {40, true}} {
		t.Run(fmt.Sprintf("z%g", row.teeth), func(t *testing.T) {
			t.Parallel()
			body := helicalToothLoft(t, involuteGear{module: 2, teeth: row.teeth, pressure: 20 * math.Pi / 180, fitPoints: 5})
			cen, err := body.Centroid()
			require.NoError(t, err)
			ratio, diameter := loftCentroidRatio(t, body, cen.Bound.Base())

			lp := body.payload.(loftPayload)
			mass, a, matchedDelta := loftCentroidRebuild(t, lp)
			radius := mass.CentroidRadius(a.verts, a.tris, cen.Value, 0)
			retired := retiredAnchorCentroidBound(t, mass, a, lp.xform.Apply(lp.plane0.Origin), matchedDelta, cen.Value)
			retiredRatio, _ := loftCentroidRatio(t, body, retired)
			anchorReach := 0.0
			for _, v := range a.verts {
				anchorReach = math.Max(anchorReach, v.Sub(lp.xform.Apply(lp.plane0.Origin)).Len())
			}
			t.Logf("R_c %.4g, tooth diameter %.4g, anchor reach %.4g; ratio %.4g, retired anchor form ratio %.4g",
				radius, diameter, anchorReach, ratio, retiredRatio)
			require.LessOrEqual(t, radius, diameter, "R_c must read the tooth's own size")
			require.Less(t, ratio, retiredRatio, "the shift form must be below the retired anchor form")
			require.Less(t, ratio, threshold, "the shift form must leave Centroid within the threshold")
			if row.retiredFails {
				require.Greater(t, retiredRatio, threshold, "the retired anchor form exceeds the threshold, which is what this test guards")
			}
		})
	}
}
