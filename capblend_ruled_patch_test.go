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

// This file is the STRAIGHT-SLANT RULED PATCH defect's own test suite
// (docs/modify-reach-design.md §8.3): a circular cap-loop-chamfer wall that
// meets a neighbour at a non-tangential miter corner has a CAP-level
// directrix (the offset arc, trimmed at the mitered corner feet) whose
// angular window differs from its SIDE-level directrix (the wall's own
// recorded sweep, untrimmed) — capblend_geom.go/capblend_moments.go used to
// read the side window for both, which moved Body.Volume() by up to 0.7% at
// a 7.24e-10 published bound. Affected exactly when a circular wall meets a
// non-tangential neighbour; unaffected and covered here too: a whole-turn
// circle, tangent junctions, and reflex corners.

// TestCapBlendCapLevelArcLengthMatchesGeometry is defect-2's public shape: a
// cap-level Arc3 edge's own Length() must equal Radius times the angle
// between its own Start() and End() (about the Arc3's own Center), within
// its own Bound — reference-free, since Start/End/Center/Radius are all
// public readings of the SAME edge, never a second construction.
// capblend_geom.go used to hold this edge's length as
// capRadius*|w.th1-w.th0| — the WALL's own full recorded sweep — rather than
// the trimmed cap-level sweep between the actual corner feet the edge runs
// between, so the held length disagreed with the edge's own endpoints.
func TestCapBlendCapLevelArcLengthMatchesGeometry(t *testing.T) {
	const R, H, d = 60.0, 20.0, 4.0
	body := quarterDiskBody(t, R, H)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)

	checked := 0
	for _, e := range chamfered.Edges() {
		arc, ok := e.Curve().(decad.Arc3)
		if !ok {
			continue
		}
		start := e.Start().Position().Value
		end := e.End().Position().Value
		if start.Z != H || end.Z != H {
			continue // not the chamfer band's cap-level directrix
		}
		checked++

		center := arc.Center
		radius := arc.Radius.Mag()
		a0 := math.Atan2(start.Y-center.Y, start.X-center.X)
		a1 := math.Atan2(end.Y-center.Y, end.X-center.X)
		want := radius * math.Abs(a1-a0)

		length, err := e.Length()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(length.Value.Mag()-want), length.Bound.Mag(),
			`cap-level arc length %v mm must enclose radius*angle(start,end) = %v mm within its own bound %v mm`,
			length.Value.Mag(), want, length.Bound.Mag())
	}
	require.Equal(t, 1, checked, `one cap-level arc: the chamfered quarter-circle wall`)
}

// simpson1D is a fixed-resolution composite Simpson's rule integrator, test
// code only: every reference below judges a residual of several mm^3 against
// a numerical integral, so float64 quadrature at this resolution (relative
// error many orders of magnitude below what it is judging) is more than
// enough — this is an independent REFERENCE, not a proof obligation the
// production code carries.
func simpson1D(f func(float64) float64, a, b float64, n int) float64 {
	if n%2 != 0 {
		n++
	}
	h := (b - a) / float64(n)
	sum := f(a) + f(b)
	for i := 1; i < n; i++ {
		x := a + float64(i)*h
		if i%2 == 0 {
			sum += 2 * f(x)
		} else {
			sum += 4 * f(x)
		}
	}
	return sum * h / 3
}

// sectorArea is a symmetric circular sector's (radius R, full angle phi)
// own area once its boundary — the arc AND the two straight radii through
// the centre — is eroded inward by t: the closed form the task's own design
// note states, A(t) = 2*integral[asin(t/(R-t)), phi/2] of
// ((R-t)^2 - (t/sin(theta))^2)/2 dtheta.
func sectorArea(R, phi, t float64) float64 {
	if t <= 0 {
		return R * R * phi / 2
	}
	lo := math.Asin(t / (R - t))
	hi := phi / 2
	if lo >= hi {
		return 0
	}
	f := func(th float64) float64 {
		s := t / math.Sin(th)
		return ((R-t)*(R-t) - s*s) / 2
	}
	return 2 * simpson1D(f, lo, hi, 4000)
}

// sectorErosionVolume is the erosion-family reference volume for a straight
// prism over a symmetric circular sector (radius R, full angle phi), swept
// h and chamfered by setback d on one cap: the straight slab below the band
// plus the band's own volume as the integral, over the eroded depth t in
// [0, d], of the sector's own area at that erosion.
func sectorErosionVolume(R, phi, h, d float64) float64 {
	slab := sectorArea(R, phi, 0) * (h - d)
	band := simpson1D(func(t float64) float64 { return sectorArea(R, phi, t) }, 0, d, 4000)
	return slab + band
}

// segmentArea is a minor circular segment's (radius R, chord at v = chordV)
// own area once its boundary is eroded inward by t: the standard closed form
// for a circular segment's area, r^2*acos(h/r) - h*sqrt(r^2-h^2), with the
// eroded radius r = R-t and the eroded chord height h = chordV+t.
func segmentArea(R, chordV, t float64) float64 {
	r := R - t
	h := chordV + t
	if h >= r {
		return 0
	}
	return r*r*math.Acos(h/r) - h*math.Sqrt(r*r-h*h)
}

// segmentErosionVolume is the erosion-family reference volume for a straight
// prism over a minor circular segment (radius R, chord at v = chordV), swept
// h and chamfered by setback d on one cap.
func segmentErosionVolume(R, chordV, h, d float64) float64 {
	slab := segmentArea(R, chordV, 0) * (h - d)
	band := simpson1D(func(t float64) float64 { return segmentArea(R, chordV, t) }, 0, d, 4000)
	return slab + band
}

// circularSegmentBody extrudes the minor circular segment cut from a
// radius-r disk (centred at the origin) by the chord v = chordV — a straight
// chord meeting the arc at two non-tangential miter corners — by h.
func circularSegmentBody(t *testing.T, r, chordV, h float64) *decad.Body {
	t.Helper()
	x0 := math.Sqrt(r*r - chordV*chordV)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	p1 := s.CreatePoint(-x0, chordV)
	p2 := s.CreatePoint(x0, chordV)
	s.CreateLine(p1, p2)
	s.CreateArc(o, p2, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// circularSectorBody extrudes a symmetric circular sector — two straight
// radii from the origin at angle 0 and phi, closed by the arc between their
// far ends — by h: a single circular wall meeting a straight neighbour at a
// non-tangential miter corner at BOTH its own ends, generalizing
// quarterDiskBody to any full angle phi.
func circularSectorBody(t *testing.T, r, phi, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(r*math.Cos(phi), r*math.Sin(phi))
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendErosionFamilyVolumeBoundEncloses is defect-1's own value check
// (docs/modify-reach-design.md §8.3): the published volume judged against the
// erosion-family integral A(0)*(H-d) + integral[0,d] of A(t) dt, an
// INDEPENDENT reference derived from the section alone (never from the
// evaluator's own construction) for shapes whose circular wall meets a
// straight neighbour at a non-tangential miter on both ends — the quarter
// disk, a minor circular segment cut by a chord well off-centre, and two
// sectors near the setback's own limit (an audited PR-122 repro: the setback
// approaching the section's own inradius makes the cap window close hard
// against the side one — capblend_moments.go's chordLocusResidualAllow judges
// exactly this residual). Every residual is the gap between the BUILT
// (straight-ruled-patch) solid and the exact miter-locus solid the erosion
// family denotes (docs/modify-reach-design.md §8.3's own scope note,
// capblend_moments.go); the published Bound must enclose it.
func TestCapBlendErosionFamilyVolumeBoundEncloses(t *testing.T) {
	for _, tc := range []struct {
		name    string
		build   func(t *testing.T) *decad.Body
		erosion func() float64
		d       float64
	}{
		{
			name:    `quarter disk`,
			build:   func(t *testing.T) *decad.Body { return quarterDiskBody(t, 60, 20) },
			erosion: func() float64 { return sectorErosionVolume(60, math.Pi/2, 20, 4) },
			d:       4,
		},
		{
			name:    `minor circular segment, chord v=30`,
			build:   func(t *testing.T) *decad.Body { return circularSegmentBody(t, 60, 30, 20) },
			erosion: func() float64 { return segmentErosionVolume(60, 30, 20, 4) },
			d:       4,
		},
		{
			name:    `quarter disk near the setback limit`,
			build:   func(t *testing.T) *decad.Body { return circularSectorBody(t, 10, math.Pi/2, 4.2) },
			erosion: func() float64 { return sectorErosionVolume(10, math.Pi/2, 4.2, 4.13) },
			d:       4.13,
		},
		{
			name:    `wide sector near the setback limit`,
			build:   func(t *testing.T) *decad.Body { return circularSectorBody(t, 10, 2.7, 4.953329) },
			erosion: func() float64 { return sectorErosionVolume(10, 2.7, 4.953329, 4.928686) },
			d:       4.928686,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tc.build(t)
			chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(tc.d))
			require.NoError(t, err)
			vol, err := chamfered.Volume()
			require.NoError(t, err)

			erosion := tc.erosion()
			residual := math.Abs(vol.Value.Mag() - erosion)
			require.LessOrEqual(t, residual, vol.Bound.Mag(),
				`the published volume bound (%v mm^3) must enclose the residual (%v mm^3) against the erosion-family reference (held %v mm^3, erosion %v mm^3)`,
				vol.Bound.Mag(), residual, vol.Value.Mag(), erosion)
		})
	}
}

// roundedRectBody extrudes an l x w rectangle by h and fillets its four
// lateral corners to radius r — a TRUE tangent fillet, so every join between
// a straight wall and a circular wall is tangential.
func roundedRectBody(t *testing.T, l, w, h, r float64) *decad.Body {
	t.Helper()
	sk := sketch.NewWorld()
	s, err := sk.CreateSketch(sk.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, l, w)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	lateral := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	rounded, err := box.Fillet(lateral, units.Millimeters(r))
	require.NoError(t, err)
	return rounded
}

// roundedRectErosionVolume is the erosion-family reference for a rounded
// rectangle (fillet radius r, tangent corners): the eroded section at depth
// t is a smaller rounded rectangle, (l-2t) x (w-2t) with fillet radius
// (r-t) — a closed form, no numerical integration needed, since a tangent
// fillet's own offset family is elementary.
func roundedRectErosionVolume(l, w, h, r, d float64) float64 {
	area0 := l*w - (4-math.Pi)*r*r
	slab := area0 * (h - d)
	rectBand := d * (l*w - (l+w)*d + (4.0/3.0)*d*d)
	filletBand := (4 - math.Pi) / 3 * (r*r*r - (r-d)*(r-d)*(r-d))
	return slab + rectBand - filletBand
}

// roundedRectHoleBody extrudes plateWithRectHole's own 100x100 plate with a
// side x side square hole (100 mm plate, 10 mm sweep, both fixed by that
// helper) and then fillets the hole's own four lateral edges to radius
// rho — a TRUE tangent fillet, so the D-HOLE this leaves (like a rounded
// rectangle's, but on a CW hole loop rather than an outer one) meets every
// straight wall and its neighbouring round wall tangentially.
func roundedRectHoleBody(t *testing.T, side, rho float64) *decad.Body {
	t.Helper()
	plate := plateWithRectHole(t, side)
	hole := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave())
	rounded, err := plate.Fillet(hole, units.Millimeters(rho))
	require.NoError(t, err)
	return rounded
}

// roundedRectHoleErosionVolume is the erosion-family reference for
// roundedRectHoleBody, BOTH the plate's own outer loop and the hole's rim
// chamfered together: the outer boundary shrinks (L-2t) x (L-2t) exactly as
// a plain rectangle's own band already does (TestCapBlendChamferPolygonLoop's
// closed form), while the rounded-rectangle hole WIDENS as the chamfer erodes
// into the material — its own side and fillet radius both growing by t, the
// same tangent-offset family a rounded rectangle's OWN outer erosion uses
// (roundedRectErosionVolume), just widening rather than shrinking.
func roundedRectHoleErosionVolume(plateL, side, rho, h, d float64) float64 {
	holeArea := func(t float64) float64 {
		s, r := side+2*t, rho+t
		return s*s - (4-math.Pi)*r*r
	}
	area0 := plateL*plateL - holeArea(0)
	slab := area0 * (h - d)
	rectBand := d * (plateL*plateL - 2*plateL*d + (4.0/3.0)*d*d)
	holeSideBand := d * (side*side + 2*side*d + (4.0/3.0)*d*d)
	holeFilletBand := (4 - math.Pi) / 3 * ((rho+d)*(rho+d)*(rho+d) - rho*rho*rho)
	holeBand := holeSideBand - holeFilletBand
	return slab + rectBand - holeBand
}

// TestCapBlendTangentJunctionVolumeUnaffected is the fix's own negative
// space: docs/modify-reach-design.md §8.3 states the defect is affected
// exactly when a circular wall meets a neighbour at a NON-tangential miter
// corner, and unaffected at a tangent junction — the cap-level and side-level
// directrices coincide there in the real numbers, so a fix that narrowed or
// widened the window unconditionally, rather than reading the actual offset
// corner feet, would show up here as a multi-mm^3 residual against the same
// closed forms these shapes already have (a rounded rectangle's fillet band,
// a D-hole's own widening) — the same order of magnitude
// TestCapBlendErosionFamilyVolumeBoundEncloses's miter cases show, never
// merely a rounding-level one.
func TestCapBlendTangentJunctionVolumeUnaffected(t *testing.T) {
	t.Run(`rounded rectangle`, func(t *testing.T) {
		const l, w, h, r, d = 100.0, 60.0, 20.0, 15.0, 4.0
		body := roundedRectBody(t, l, w, h, r)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		want := roundedRectErosionVolume(l, w, h, r, d)
		require.InDelta(t, want, vol.Value.Mag(), 1e-6)
	})

	t.Run(`D-hole`, func(t *testing.T) {
		const plateL, side, rho, h, d = 100.0, 20.0, 3.0, 10.0, 2.0
		body := roundedRectHoleBody(t, side, rho)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		want := roundedRectHoleErosionVolume(plateL, side, rho, h, d)
		require.InDelta(t, want, vol.Value.Mag(), 1e-6)
	})
}

// TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight pins the two
// PR-122 audit reproductions defect-3 fixed: a shape whose every circular
// wall meets its neighbour tangentially (so the two directrices share one
// window, `chordLocusResidualAllow` contributes nothing, and the ONLY thing
// left in play is the arithmetic-rounding envelope itself), and the
// cornerless whole-turn circle. `patchRawFlux`'s poly/cross terms used to
// read the absolute plane-local levels z0, z1 rather than the band's own z
// origin, which left the published Bound scale with those absolute levels
// instead of the band's own small axial extent — a restructure sound enough
// to still enclose the residual (Bound only ever WIDENS, never wrongly
// shrinks) but far looser than it needs to be on a body that never asked for
// the cap-loop chamfer's own general non-tangent-miter machinery at all. The
// ceilings below are well above the measured value (so ordinary arithmetic
// reordering cannot flake this) and well below the values a bound regressed
// back to reading the absolute levels would publish (audited at
// 28386.2821001117 and 1.450768679e-08 respectively).
func TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight(t *testing.T) {
	t.Run(`tangent-fillet plate`, func(t *testing.T) {
		const l, w, h, r, d = 100.0, 60.0, 20.0, 15.0, 4.0
		body := roundedRectBody(t, l, w, h, r)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 20000.0,
			`the tangent-junction bound (%v mm^3) must stay close to the pre-PR-122 reading (15600.0000000006 mm^3), not the audited regression (28386.2821001117 mm^3)`,
			vol.Bound.Mag())
	})

	t.Run(`whole-turn circle`, func(t *testing.T) {
		body := circleProfile(t, 60, 20)
		chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(4))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 5e-9,
			`the whole-turn bound (%v mm^3) must stay close to the pre-PR-122 reading (2.743387239e-09 mm^3), not the audited regression (1.450768679e-08 mm^3)`,
			vol.Bound.Mag())
	})
}

// dHoleBody extrudes a 100x100 plate with a "D" shaped hole: a circle of
// radius r centred at (cx, cy), cut by a chord between the points at
// phi0Deg and phi1Deg (degrees, measured from the centre) — a straight wall
// meeting a circular wall at two non-tangential miter corners, on a HOLE
// loop rather than an outer one. minor picks which of the chord's two arcs
// is the hole's own wall: true keeps the SHORT (<180 degrees) arc, false
// the LONG (>180 degrees) one — the major-arc case is PR-122's own audited
// branch-crossing repro (capWallSweep's raw Atan2 lands the offset foot's
// angle a full turn from the wall's own recorded th0).
func dHoleBody(t *testing.T, cx, cy, r, phi0Deg, phi1Deg float64, minor bool) *decad.Body {
	t.Helper()
	phi0 := phi0Deg * math.Pi / 180
	phi1 := phi1Deg * math.Pi / 180
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	o := s.CreatePoint(cx, cy)
	p0 := s.CreatePoint(cx+r*math.Cos(phi0), cy+r*math.Sin(phi0))
	p1 := s.CreatePoint(cx+r*math.Cos(phi1), cy+r*math.Sin(phi1))
	s.CreateLine(p0, p1)
	if minor {
		s.CreateArc(o, p0, p1)
	} else {
		s.CreateArc(o, p1, p0)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof, `the rectangle-with-D-hole region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestCapBlendDHoleLoopBoundStaysTight is the fix's own hole-loop floor:
// TestCapBlendTangentJunctionAndWholeTurnBoundsStayTight above pins the
// tight-bound property for an outer-loop tangent fillet and a whole-turn
// circle, but neither is a HOLE loop and neither has a genuinely
// non-tangential miter corner, so neither could have caught PR-122's own
// defect (a D-hole whose wall is the MAJOR arc: capWallSweep's raw Atan2
// puts the offset foot's angle a full 2*pi from the wall's own recorded
// th0, so chordLocusResidualAllow's windowSkewMax read close to 2*pi
// instead of the corner's own small miter skew, and the published bound
// inflated by orders of magnitude on a patch whose true residual is
// negligible). The major-arc case below is that exact repro, audited at a
// pre-fix bound of 22972.28860508683 mm^3 (windowSkewMax == 2*pi, up to
// float rounding) against a post-fix bound of 7157.192745203181 mm^3; the
// minor-arc case is the same hole family with the wall on the SHORT arc
// instead, which never crosses branches and never needed the fix, given a
// floor of its own so a future change to either patch cannot silently
// regress it unnoticed.
func TestCapBlendDHoleLoopBoundStaysTight(t *testing.T) {
	t.Run(`major-arc hole (branch-crossing repro)`, func(t *testing.T) {
		body := dHoleBody(t, 50, 50, 10, 30, 150, false)
		chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 10000.0,
			`the major-arc D-hole bound (%v mm^3) must stay close to the post-fix reading (7157.192745203181 mm^3), not the pre-fix branch-mismatch regression (22972.28860508683 mm^3)`,
			vol.Bound.Mag())
	})

	t.Run(`minor-arc hole`, func(t *testing.T) {
		body := dHoleBody(t, 50, 50, 10, 30, 150, true)
		chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Less(t, vol.Bound.Mag(), 5000.0,
			`the minor-arc D-hole bound (%v mm^3) must stay close to the audited reading (2733.0793750625394 mm^3)`,
			vol.Bound.Mag())
	})
}
