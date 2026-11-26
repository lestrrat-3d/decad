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

// This file covers a union whose plate profile has holes and whose other operand shares a span of
// a hole's wall (a pillar of the hole's own radius, or a block whose wall lies in the hole's wall
// plane). The result is analytic when the shared span resolves, and keeps the mesh path otherwise.

// spokedHubDisc extrudes a disc of radius 21, 5 thick from z = 0, with four windows between arms 4 wide. Each
// window's inner wall is an arc of radius hub about the axis, and every window corner is rounded with radius 2.
func spokedHubDisc(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	const (
		hub      = 7.0
		rimInner = 18.0
		armHalf  = 2.0
		round    = 2.0
	)
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	o := s.Origin()
	x1 := s.CreatePoint(10, 0)
	ref := s.CreateLine(o, x1)
	ref.SetConstruction(true)
	s.AddConstraint(sketch.NewHorizontal(ref), sketch.NewDistance(o, x1, 10))
	rim := s.CreateCircle(o, 21)
	s.AddConstraint(sketch.NewRadius(rim, 21))
	for k := range 4 {
		s1 := math.Pi/4 + math.Pi/2*float64(k)
		s2 := s1 + math.Pi/2
		di, do := math.Asin(armHalf/hub), math.Asin(armHalf/rimInner)
		a := s.CreatePoint(hub*math.Cos(s1+di), hub*math.Sin(s1+di))
		bb := s.CreatePoint(hub*math.Cos(s2-di), hub*math.Sin(s2-di))
		c := s.CreatePoint(rimInner*math.Cos(s2-do), rimInner*math.Sin(s2-do))
		d := s.CreatePoint(rimInner*math.Cos(s1+do), rimInner*math.Sin(s1+do))
		inner := s.CreateArc(o, a, bb)
		outer := s.CreateArc(o, d, c)
		side1 := s.CreateLine(a, d)
		side2 := s.CreateLine(bb, c)
		s.AddConstraint(
			sketch.NewRadius(inner, hub), sketch.NewRadius(outer, rimInner),
			sketch.NewAngle(ref, side1, s1*180/math.Pi), sketch.NewDistancePointLine(o, side1, armHalf),
			sketch.NewAngle(ref, side2, s2*180/math.Pi), sketch.NewDistancePointLine(o, side2, armHalf),
		)
		_, err = sketch.CreateFillet(s, inner, side1, a, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, side2, inner, bb, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, outer, side2, c, round)
		require.NoError(t, err)
		_, err = sketch.CreateFillet(s, side1, outer, d, round)
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 4 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	body, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// bossOnCap puts a round boss at the origin of the body's end cap, which lies at z = 5, 1 mm tall.
func bossOnCap(t *testing.T, body *decad.Body, diameter float64) (*decad.Body, error) {
	t.Helper()
	face := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).Exactly(1)
	return decad.Boss(t.Context(), body, decad.BossSpec{Face: face, At: r3.Vec{Z: 5},
		Diameter: units.Millimeters(diameter), Height: units.Millimeters(1)})
}

// volumeAndBound reads a body's volume and its error bound in mm³.
func volumeAndBound(t *testing.T, b *decad.Body) (float64, float64) {
	t.Helper()
	m, err := b.Volume()
	require.NoError(t, err)
	v, err := m.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := m.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v, bound
}

// requireAnalytic requires that no edge of the body is faceted and that every face surface is a plane or a cylinder.
func requireAnalytic(t *testing.T, b *decad.Body) {
	t.Helper()
	for _, e := range b.Edges() {
		_, faceted := e.Curve().(decad.FacetedCurve)
		require.False(t, faceted, "edge %v -> %v is faceted", e.Start().Position().Value, e.End().Position().Value)
	}
	for _, f := range b.Faces() {
		switch f.Surface().(type) {
		case decad.Plane, decad.Cylinder:
		default:
			require.Failf(t, "surface", "face has surface %T", f.Surface())
		}
	}
}

// isFaceted reports whether any edge of the body is faceted, which is the mesh route's mark.
func isFaceted(b *decad.Body) bool {
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.FacetedCurve); ok {
			return true
		}
	}
	return false
}

// faceZSpan returns the lowest and highest vertex z over a face's edges.
func faceZSpan(f *decad.Face) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, e := range f.Edges() {
		for _, z := range []float64{e.Start().Position().Value.Z, e.End().Position().Value.Z} {
			lo, hi = math.Min(lo, z), math.Max(hi, z)
		}
	}
	return lo, hi
}

// requireRootArcs requires that the planar face at z holds exactly n reflex Arc3 edges of the given
// radius about the z axis, and returns that face.
func requireRootArcs(t *testing.T, b *decad.Body, z, radius float64, n int) {
	t.Helper()
	var faces []*decad.Face
	for _, f := range b.Faces() {
		p, ok := f.Surface().(decad.Plane)
		if ok && math.Abs(p.Frame.Origin().Z-z) < 1e-9 && math.Abs(math.Abs(p.Frame.N().Z)-1) < 1e-9 {
			faces = append(faces, f)
		}
	}
	require.Len(t, faces, 1, "planar faces at z = %g", z)
	arcs := 0
	for _, f := range faces {
		for _, e := range f.Edges() {
			_, faceted := e.Curve().(decad.FacetedCurve)
			require.False(t, faceted)
			a, ok := e.Curve().(decad.Arc3)
			if !ok {
				continue
			}
			r, err := a.Radius.In(units.Millimeter)
			require.NoError(t, err)
			if math.Abs(r-radius) > 1e-12 {
				continue
			}
			require.InDelta(t, 0, a.Center.X, 1e-12)
			require.InDelta(t, 0, a.Center.Y, 1e-12)
			require.Equal(t, decad.WedgeReflex, e.Wedge())
			arcs++
		}
	}
	require.Equal(t, n, arcs)
}

func TestUnionHoledSharedSpanBoss(t *testing.T) {
	t.Parallel()
	doc, disc := spokedHubDisc(t)
	discVolume, discBound := volumeAndBound(t, disc)
	out, err := bossOnCap(t, disc, 14)
	require.NoError(t, err)

	requireAnalytic(t, out)
	requireRootArcs(t, out, 5, 7, 4)

	var full, upper int
	for _, f := range out.Faces() {
		c, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		r, err := c.Radius.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(r-7) > 1e-12 || math.Abs(c.Origin.X) > 1e-12 || math.Abs(c.Origin.Y) > 1e-12 {
			continue
		}
		require.InDelta(t, 1, math.Abs(c.Axis.Z), 1e-12)
		lo, hi := faceZSpan(f)
		switch {
		case math.Abs(lo) < 1e-9 && math.Abs(hi-6) < 1e-9:
			full++
		case math.Abs(lo-5) < 1e-9 && math.Abs(hi-6) < 1e-9:
			upper++
		default:
			require.Failf(t, "cylinder", "radius 7 cylinder spans z %v to %v", lo, hi)
		}
	}
	require.Equal(t, 4, full, "radius 7 cylinders spanning z 0 to 6")
	require.Equal(t, 4, upper, "radius 7 cylinders spanning z 5 to 6")

	v, bound := volumeAndBound(t, out)
	t.Logf("volume %.17g bound %g faces %d, disc %.17g bound %g", v, bound, len(out.Faces()), discVolume, discBound)
	require.LessOrEqual(t, math.Abs(v-(discVolume+math.Pi*49)), bound+discBound)
	require.Less(t, bound, 1e-6)

	for _, c := range []struct {
		p    r3.Vec
		want decad.PointLocation
	}{
		{r3.Vec{X: 0, Y: 10, Z: 2.5}, decad.PointOutside},
		{r3.Vec{X: 3, Y: 0, Z: 2.5}, decad.PointInside},
	} {
		got, err := out.LocatePoint(t.Context(), c.p, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, c.want, got, "point %v", c.p)
	}

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
	_, err = out.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
}

func TestUnionHoledNoSharedSpanBossKeepsMeshPath(t *testing.T) {
	t.Parallel()
	for _, c := range []struct {
		diameter float64
		faces    int
	}{{12, 37}, {13, 37}, {15, 41}} {
		_, disc := spokedHubDisc(t)
		out, err := bossOnCap(t, disc, c.diameter)
		require.NoError(t, err, "diameter %g", c.diameter)
		_, bound := volumeAndBound(t, out)
		t.Logf("diameter %g: faces %d bound %g", c.diameter, len(out.Faces()), bound)
		require.Len(t, out.Faces(), c.faces, "diameter %g", c.diameter)
		require.Greater(t, bound, 0.5, "diameter %g", c.diameter)
	}
}

func TestUnionHoledSharedSpanKidneySlot(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-14, -11, 26, 19)
	s.Fix(rect.A)
	const cx, cy = 6.0, 4.0
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	slot, err := s.CreateArcSlot(cx, cy, cx+7*math.Cos(rad(30)), cy+7*math.Sin(rad(30)),
		cx+7*math.Cos(rad(150)), cy+7*math.Sin(rad(150)), 2)
	require.NoError(t, err)
	s.Fix(slot.Center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	t.Logf("slot centre (%v, %v)", slot.Center.X(), slot.Center.Y())
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	plate, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	plateVolume, plateBound := volumeAndBound(t, plate)

	face := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))).Exactly(1)
	out, err := decad.Boss(t.Context(), plate, decad.BossSpec{Face: face, At: r3.Vec{X: cx, Y: cy, Z: 4},
		Diameter: units.Millimeters(10), Height: units.Millimeters(2)})
	require.NoError(t, err)
	requireAnalytic(t, out)
	v, bound := volumeAndBound(t, out)
	t.Logf("volume %.17g bound %g faces %d, plate %.17g", v, bound, len(out.Faces()), plateVolume)
	require.LessOrEqual(t, math.Abs(v-(plateVolume+math.Pi*25*2)), bound+plateBound)
	require.Less(t, bound, 1e-6)

	// The root arcs are about the slot's centre, not the origin.
	var roots int
	for _, f := range out.Faces() {
		p, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(p.Frame.Origin().Z-4) > 1e-9 || math.Abs(math.Abs(p.Frame.N().Z)-1) > 1e-9 {
			continue
		}
		for _, e := range f.Edges() {
			a, ok := e.Curve().(decad.Arc3)
			if !ok {
				continue
			}
			r, err := a.Radius.In(units.Millimeter)
			require.NoError(t, err)
			if math.Abs(r-5) > 1e-12 {
				continue
			}
			require.InDelta(t, cx, a.Center.X, 1e-9)
			require.InDelta(t, cy, a.Center.Y, 1e-9)
			require.Equal(t, decad.WedgeReflex, e.Wedge())
			roots++
		}
	}
	require.Positive(t, roots, "reflex root arcs of radius 5 on the z = 4 face")
}

// holedSquarePlate extrudes a [-20, 20]² plate with a [-5, 5]² square hole from z = 0 to z = 5.
func holedSquarePlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(outer.A)
	inner := s.CreateRectangle(-5, -5, 5, 5)
	s.Fix(inner.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			body, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
			require.NoError(t, err)
			return body
		}
	}
	t.Fatal("the plate sketch has no holed profile")
	return nil
}

func TestUnionHoledSharedSpanExactLines(t *testing.T) {
	t.Parallel()
	// Scaling retires its input, so it builds its own copy.
	build := func() *decad.Body {
		doc := decad.New()
		plate := holedSquarePlate(t, doc)
		block := boxBodyAtZ(t, doc, -8, 5, 8, 15, 4, 2)
		out, err := decad.Union(plate, block).Do(t.Context())
		require.NoError(t, err)
		return out
	}
	out := build()

	want := 40.0*40*5 - 10*10*5 + 16*10*1
	m, err := out.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, m.Exactness)
	v, err := m.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Equal(t, want, v)

	var wall []*decad.Face
	for _, f := range out.Faces() {
		p, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(p.Frame.Origin().Y-5) > 1e-9 || math.Abs(math.Abs(p.Frame.N().Y)-1) > 1e-9 {
			continue
		}
		n, err := f.NormalAt(r3.Vec{Y: 5, Z: 2.5})
		require.NoError(t, err)
		if n.Value.Y < -0.5 {
			wall = append(wall, f)
		}
	}
	require.Len(t, wall, 1, "faces in the plane y = 5 facing -y")
	lo, hi := faceZSpan(wall[0])
	require.InDelta(t, 0, lo, 1e-12)
	require.InDelta(t, 6, hi, 1e-12)

	forScale := build()
	scaled, err := decad.ScaledAt(t.Context(), forScale, r3.Vec{}, units.Scalar(2))
	require.NoError(t, err)
	sv, sb := volumeAndBound(t, scaled)
	require.LessOrEqual(t, math.Abs(sv-8*want), sb)

	frame := mustFrame(t, r3.Vec{Z: 2.5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	_, profiles, err := out.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 1)

	for _, c := range []struct {
		p    r3.Vec
		want decad.PointLocation
	}{
		{r3.Vec{Z: 2.5}, decad.PointOutside},
		{r3.Vec{X: 10, Z: 2.5}, decad.PointInside},
		{r3.Vec{Y: 10, Z: 5.5}, decad.PointInside},
	} {
		got, err := out.LocatePoint(t.Context(), c.p, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, c.want, got, "point %v", c.p)
	}

	// A later Union reads the holed stack as an A1 operand. This one passes
	// the gate (the second block's wall lies on the hole's other wall), but the
	// interface scene at z = 5 holds two records of the first result that share
	// the hole's top wall, which the build does not cover, so it takes the mesh
	// path with no error and a volume within its bound.
	second := boxBodyAtZ(t, out.Document(), -8, -15, 8, -5, 4, 2)
	joined, err := decad.Union(out, second).Do(t.Context())
	require.NoError(t, err)
	jv, jb := volumeAndBound(t, joined)
	require.LessOrEqual(t, math.Abs(jv-(want+16*10*1)), jb)
}

func TestUnionHoledSharedSpanSharpSector(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.Origin()
	rim := s.CreateCircle(o, 20)
	s.AddConstraint(sketch.NewRadius(rim, 20))
	at := func(r, deg float64) *sketch.Point {
		a := deg * math.Pi / 180
		return s.CreatePoint(r*math.Cos(a), r*math.Sin(a))
	}
	i1, i2, o1, o2 := at(7, 45), at(7, 135), at(15, 45), at(15, 135)
	inner := s.CreateArc(o, i1, i2)
	outer := s.CreateArc(o, o1, o2)
	s.CreateLine(i1, o1)
	s.CreateLine(i2, o2)
	s.AddConstraint(sketch.NewRadius(inner, 7), sketch.NewRadius(outer, 15))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	disc, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	discVolume, discBound := volumeAndBound(t, disc)

	out, err := bossOnCap(t, disc, 14)
	require.NoError(t, err)
	requireAnalytic(t, out)
	v, bound := volumeAndBound(t, out)
	t.Logf("volume %.17g bound %g faces %d, disc %.17g", v, bound, len(out.Faces()), discVolume)
	require.LessOrEqual(t, math.Abs(v-(discVolume+math.Pi*49)), bound+discBound)
	require.Less(t, bound, 1e-6)
}

func TestUnionHoledSharedSpanScaleRefusesCharged(t *testing.T) {
	t.Parallel()
	_, disc := spokedHubDisc(t)
	out, err := bossOnCap(t, disc, 14)
	require.NoError(t, err)
	requireAnalytic(t, out)

	_, err = decad.ScaledAt(t.Context(), out, r3.Vec{}, units.Scalar(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	frame := mustFrame(t, r3.Vec{Z: 2.5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	_, _, err = out.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	// The pillar's top face takes a further boss.
	var top *decad.Face
	for _, f := range out.Faces() {
		if p, ok := f.Surface().(decad.Plane); ok && math.Abs(p.Frame.Origin().Z-6) < 1e-9 {
			top = f
		}
	}
	require.NotNil(t, top)
	require.NotEmpty(t, top.Origins())
	t.Logf("top face origins %v", top.Origins())
	selector := decad.Faces(decad.FaceCreatedBy(top.Origins()[0])).Exactly(1)
	_, err = decad.Boss(t.Context(), out, decad.BossSpec{Face: selector, At: r3.Vec{Z: 6},
		Diameter: units.Millimeters(4), Height: units.Millimeters(1)})
	require.NoError(t, err)
}

// TestUnionHoledSharedSpanNearEqualPillarChargesItsWidth grows the pillar by a few ulps past the windows'
// inner arcs (recorded at 7 and 7.0000000000000009). sketch still resolves each shared span, so the union
// stays analytic, and the span's width enters every face's displacement: the published volume bound grows
// with the gap between the pillar and the arcs.
func TestUnionHoledSharedSpanNearEqualPillarChargesItsWidth(t *testing.T) {
	t.Parallel()
	prev := 0.0
	for _, d := range []float64{14, 14.000000000000004, 14.00000000000001} {
		_, disc := spokedHubDisc(t)
		out, err := bossOnCap(t, disc, d)
		require.NoError(t, err, "diameter %.17g", d)
		require.False(t, isFaceted(out), "diameter %.17g", d)
		_, bound := volumeAndBound(t, out)
		require.Less(t, bound, 1e-6, "diameter %.17g", d)
		require.Greater(t, bound, prev, "diameter %.17g: the wider span must widen the bound", d)
		prev = bound
	}
}
