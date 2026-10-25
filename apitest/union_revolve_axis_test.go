package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// revolveAxisPlacement maps the hubs, cone tools and radial lug of the +X
// sector Union tests onto one axis through the origin (agent-docs/revolve-axis-design.md
// §6). A design point (x, r) of a meridian sketch is the world point x·Â + r·N̂,
// where Â is the unit axis and N̂ its left normal in the sketch plane. A design
// world vector maps by X → Â, Y → N̂, Z → Â × N̂.
type revolveAxisPlacement struct {
	// plane is the frame of the blank and cone tool sketches, or nil for the
	// world's XY datum plane.
	plane *r3.Frame
	// axisEnd is the end of the SketchLine from (0, 0) the blank and the tools
	// revolve about.
	axisEnd decad.Point2
	// onAxis returns the plane point of design point (x, 0), exactly on the
	// axis line, and offAxis that of (x, r).
	onAxis  func(x float64) decad.Point2
	offAxis func(x, r float64) decad.Point2
	// world maps a design world vector.
	world func(v r3.Vec) r3.Vec
	// axis is the float unit axis the copies turn about.
	axis r3.Vec
}

func xAxisPlacement(t *testing.T) revolveAxisPlacement {
	t.Helper()
	return revolveAxisPlacement{axisEnd: decad.Point2{U: 1},
		onAxis:  func(x float64) decad.Point2 { return decad.Point2{U: x} },
		offAxis: func(x, r float64) decad.Point2 { return decad.Point2{U: x, V: r} },
		world:   func(v r3.Vec) r3.Vec { return v },
		axis:    r3.Vec{X: 1}}
}

// permutedAxisPlacement maps every input by the exact rotation
// (x, y, z) → (z, x, y), so the axis is (0, 1, 0) and the result is congruent
// to the +X one with no rounding.
func permutedAxisPlacement(t *testing.T) revolveAxisPlacement {
	t.Helper()
	plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	return revolveAxisPlacement{plane: &plane, axisEnd: decad.Point2{U: 1},
		onAxis:  func(x float64) decad.Point2 { return decad.Point2{U: x} },
		offAxis: func(x, r float64) decad.Point2 { return decad.Point2{U: x, V: r} },
		world:   func(v r3.Vec) r3.Vec { return r3.Vec{X: v.Z, Y: v.X, Z: v.Y} },
		axis:    r3.Vec{Y: 1}}
}

// xzAxisPlacement sketches on the world XZ plane and revolves about its U
// axis, world X: the blank misses the frame predicate, so its certificate
// takes the general record branch, while its axis value is X and every trim
// takes the +X arithmetic (agent-docs/revolve-axis-design.md §3.3). Every
// input is mapped by the exact quarter turn (x, y, z) → (x, −z, y).
func xzAxisPlacement(t *testing.T) revolveAxisPlacement {
	t.Helper()
	plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	return revolveAxisPlacement{plane: &plane, axisEnd: decad.Point2{U: 1},
		onAxis:  func(x float64) decad.Point2 { return decad.Point2{U: x} },
		offAxis: func(x, r float64) decad.Point2 { return decad.Point2{U: x, V: r} },
		world:   func(v r3.Vec) r3.Vec { return r3.Vec{X: v.X, Y: -v.Z, Z: v.Y} },
		axis:    r3.Vec{X: 1}}
}

// tiltedAxisPlacement sketches on the world XY plane about the line from the
// origin to dir, so the axis is A = (dir, 0). A design point (x, r) becomes
// s·dir + q·n with n = (−dir.Y, dir.X), where s = x/|dir| and q = r/|dir| are
// each rounded to a multiple of grid. grid is coarse enough that both plane
// coordinates are exact, so the point's A-scaled readings are exactly
// Z = s·|A|² and R = q·|A|²: an on-axis point lies exactly on the axis line,
// and a cone tool's rim lies exactly on its far plane, as the consumer
// contract requires (agent-docs/revolve-axis-design.md §6). Each point moves
// at most |dir|·grid from its exactly rotated position.
func tiltedAxisPlacement(t *testing.T, dir r3.Vec, grid float64) revolveAxisPlacement {
	t.Helper()
	norm := dir.Len()
	a := dir.Scale(1 / norm)
	n := r3.Vec{X: -a.Y, Y: a.X}
	round := func(v float64) float64 { return math.Round(v/norm/grid) * grid }
	offAxis := func(x, r float64) decad.Point2 {
		s, q := round(x), round(r)
		return decad.Point2{U: s*dir.X - q*dir.Y, V: s*dir.Y + q*dir.X}
	}
	return revolveAxisPlacement{axisEnd: decad.Point2{U: dir.X, V: dir.Y},
		onAxis:  func(x float64) decad.Point2 { return offAxis(x, 0) },
		offAxis: offAxis,
		world:   func(v r3.Vec) r3.Vec { return a.Scale(v.X).Add(n.Scale(v.Y)).Add(r3.Vec{Z: v.Z}) },
		axis:    a}
}

// round26 rounds v to 26 significant bits.
func round26(v float64) float64 {
	frac, exp := math.Frexp(v)
	return math.Ldexp(math.Round(math.Ldexp(frac, 26)), exp-26)
}

// revolveAbout revolves a closed polygon of design points about the axis.
// Points with r = 0 go through onAxis.
func (p revolveAxisPlacement) revolveAbout(t *testing.T, doc *decad.Document, outline [][2]float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	plane := world.XY()
	if p.plane != nil {
		var err error
		plane, err = world.CreatePlaneFromFrame(*p.plane)
		require.NoError(t, err)
	}
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(outline))
	for i, design := range outline {
		at := p.offAxis(design[0], design[1])
		if design[1] == 0 {
			at = p.onAxis(design[0])
		}
		points[i] = s.CreatePoint(at.U, at.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := decad.Revolve(t.Context(), doc, s, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: p.axisEnd}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// The hub outlines of the sector Union tests in design points (x, r): the
// shouldered hub, and the tapered hub whose two planar caps the bore Cut
// reads. The tapered hub draws its high cap as one line; split at (9, 1.5),
// as the +X fixture taperedHubAndRadialLug draws it, its join about (1, 2, 0)
// takes ordinary dispatch.
var (
	shoulderedHubOutline = [][2]float64{{6, 0}, {9, 0}, {9.2, 1.5}, {9, 3}, {8, 4}, {5, 2.5}, {6, 1.5}}
	taperedHubOutline    = [][2]float64{{6, 0}, {9, 0}, {9, 3}, {8, 4}, {5, 2.5}, {6, 1.5}}
)

// coneTool is pointConeTool about the axis: the cone of slope 1 whose apex
// lies the given distance along the axis, with its far cap 2 mm behind the
// origin.
func (p revolveAxisPlacement) coneTool(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
	t.Helper()
	return p.revolveAbout(t, doc, [][2]float64{{-2, 0}, {-2, apex + 2}, {apex, 0}})
}

// lug is the trimmed radial lug of the +X sector Union tests: lofted from the
// origin, cut by the toe cone at 7.5 mm and intersected with the heel cone at
// 12 mm.
func (p revolveAxisPlacement) lug(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	const rootRadius, half = 4.3, 0.4
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(p.world(r3.Vec{X: 10}), p.world(r3.Vec{Y: -1}), p.world(r3.Vec{Z: -1}))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rootLow, flankLow := s.CreatePoint(-rootRadius, -half), s.CreatePoint(-5.5, -half)
	tipLow, tipHigh := s.CreatePoint(-6, -half), s.CreatePoint(-6, half)
	flankHigh, rootHigh := s.CreatePoint(-5.5, half), s.CreatePoint(-rootRadius, half)
	center := s.CreatePoint(0, 0)
	for _, point := range []*sketch.Point{rootLow, flankLow, tipLow, tipHigh, flankHigh, rootHigh, center} {
		s.Fix(point)
	}
	s.CreateLine(rootLow, flankLow)
	s.CreateLine(flankLow, tipLow)
	s.CreateArc(center, tipHigh, tipLow)
	s.CreateLine(tipHigh, flankHigh)
	s.CreateLine(flankHigh, rootHigh)
	s.CreateArc(center, rootHigh, rootLow)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	lug, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, profiles[0])
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), lug, p.coneTool(t, doc, 7.5))
	require.NoError(t, err)
	trimmed, err := decad.Intersect(t.Context(), cut, p.coneTool(t, doc, 12))
	require.NoError(t, err)
	return trimmed
}

// revolveAxisUnion is one joined hub and its lugs about one axis, with the
// hub's own volume.
type revolveAxisUnion struct {
	doc        *decad.Document
	joined     *decad.Body
	mesh       *decad.Mesh
	volume     decad.Measurement
	hubVolume  decad.Measurement
	joinedArea decad.Measurement
	hub        *decad.Body
}

// union revolves the hub outline, builds the trimmed lug, copies the lug
// about the axis by each of turns, and joins them all, serially or in one
// variadic Union.
func (p revolveAxisPlacement) union(t *testing.T, outline [][2]float64, turns []float64,
	serial bool) revolveAxisUnion {
	t.Helper()
	doc := decad.New()
	hub := p.revolveAbout(t, doc, outline)
	lug := p.lug(t, doc)
	operands := []*decad.Body{hub, lug}
	for _, degrees := range turns {
		turn, err := r3.Rotation(p.axis, units.Degrees(degrees))
		require.NoError(t, err)
		placed, err := decad.Copy(t.Context(), lug, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		operands = append(operands, placed)
	}
	var joined *decad.Body
	var err error
	if serial {
		joined = operands[0]
		for _, operand := range operands[1:] {
			joined, err = decad.Union(t.Context(), joined, operand)
			require.NoError(t, err)
		}
	} else {
		joined, err = decad.Union(t.Context(), operands[0], operands[1:]...)
		require.NoError(t, err)
	}
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
	requireClosedMesh(t, mesh)
	volume, err := joined.Volume()
	require.NoError(t, err)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	area, err := joined.Area()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), hubVolume.Value.Base()+hubVolume.Bound.Base())
	return revolveAxisUnion{doc: doc, joined: joined, mesh: mesh, volume: volume, hubVolume: hubVolume,
		joinedArea: area, hub: hub}
}

// requireClosedMesh checks that every edge of mesh borders exactly two
// triangles, once in each direction.
func requireClosedMesh(t *testing.T, mesh *decad.Mesh) {
	t.Helper()
	directed := make(map[[2]int]int)
	for _, triangle := range mesh.Triangles() {
		for i := range 3 {
			directed[[2]int{triangle[i], triangle[(i+1)%3]}]++
		}
	}
	for edge, count := range directed {
		require.Equal(t, 1, count, "edge %v is used %d times in one direction", edge, count)
		require.Equal(t, 1, directed[[2]int{edge[1], edge[0]}], "edge %v has no opposite", edge)
	}
}

// requireVolumesAgree checks that two volume readings of congruent solids
// overlap: their difference is at most the sum of their proven bounds plus
// slack, the most the inputs' own rounding can move a volume.
func requireVolumesAgree(t *testing.T, a, b decad.Measurement, slack float64) {
	t.Helper()
	diff := math.Abs(a.Value.Base() - b.Value.Base())
	allowed := a.Bound.Base() + b.Bound.Base() + slack
	require.LessOrEqual(t, diff, allowed, "volumes %.12g and %.12g differ by %.3g, bounds %.3g and %.3g",
		a.Value.Base(), b.Value.Base(), diff, a.Bound.Base(), b.Bound.Base())
}

// inputSlack bounds how far a volume moves when every input point of a solid
// with surface area at most area moves by at most shift millimetres.
func inputSlack(area decad.Measurement, shift float64) float64 {
	return (area.Value.Base() + area.Bound.Base()) * shift
}

// The hub and three copied lugs about (0, 1, 0) are the +X construction mapped
// by an exact coordinate permutation, so the two joined solids are congruent.
// Their volumes, and the hubs' volumes, agree within the proven bounds with no
// input slack (agent-docs/revolve-axis-design.md §7.5).
func TestUnionHubAndRadialLugsAboutPermutedAxis(t *testing.T) {
	turns := []float64{120, 240}
	x := xAxisPlacement(t).union(t, shoulderedHubOutline, turns, false)
	for name, placement := range map[string]revolveAxisPlacement{
		"(0,1,0)":          permutedAxisPlacement(t),
		"XZ plane about X": xzAxisPlacement(t),
	} {
		congruent := placement.union(t, shoulderedHubOutline, turns, false)
		requireVolumesAgree(t, x.volume, congruent.volume, 0)
		requireVolumesAgree(t, x.hubVolume, congruent.hubVolume, 0)
		t.Logf("%s: +X volume %.12g ± %.3g, volume %.12g ± %.3g", name, x.volume.Value.Base(),
			x.volume.Bound.Base(), congruent.volume.Value.Base(), congruent.volume.Bound.Base())
	}
}

// A hub and three copied lugs about an axis of irrational length join through
// the public Union, serially and in one variadic call. Each result is
// watertight with Euler characteristic 2 and passes VerifyAll, and its volume
// agrees with the same construction about +X within the proven bounds
// (agent-docs/revolve-axis-design.md §7.5). The axes are (1, 2, 0), and the
// gallery's float direction: (1, 2)/√5 rounded to 26 significant bits.
//
// The inputs move by at most |dir|·grid (tiltedAxisPlacement), the lug's
// plane frame rounds by a few units in the last place, and each copy turns
// within massmoment.DriftSinLimit = 2⁻⁴⁶ of the axis: below 1e-8 mm on a
// 2⁻³⁰ grid and 1e-6 mm on a 2⁻²² grid.
func TestUnionHubAndRadialLugsAboutTiltedAxis(t *testing.T) {
	turns := []float64{120, 240}
	x := xAxisPlacement(t).union(t, shoulderedHubOutline, turns, false)
	s5 := math.Sqrt(5)
	for _, tc := range []struct {
		name        string
		dir         r3.Vec
		grid, shift float64
		serialToo   bool
	}{
		{"(1,2,0)", r3.Vec{X: 1, Y: 2}, 0x1p-30, 1e-8, true},
		{"26-bit (1,2)/√5", r3.Vec{X: round26(1 / s5), Y: round26(2 / s5)}, 0x1p-22, 1e-6, false},
	} {
		for _, serial := range []bool{false, true} {
			if serial && !tc.serialToo {
				continue
			}
			tilted := tiltedAxisPlacement(t, tc.dir, tc.grid).union(t, shoulderedHubOutline, turns, serial)
			slack := inputSlack(tilted.joinedArea, tc.shift)
			requireVolumesAgree(t, x.volume, tilted.volume, slack)
			requireVolumesAgree(t, x.hubVolume, tilted.hubVolume, slack)
			t.Logf("%s serial %v: +X volume %.12g ± %.3g, volume %.12g ± %.3g, hub %.12g ± %.3g", tc.name,
				serial, x.volume.Value.Base(), x.volume.Bound.Base(), tilted.volume.Value.Base(),
				tilted.volume.Bound.Base(), tilted.hubVolume.Value.Base(), tilted.hubVolume.Bound.Base())
		}
	}
}

// boreTool extrudes a circle of the given radius about the axis from 4 mm to
// 10 mm along it: revolveSectorCylinderTool mapped onto the placement.
func (p revolveAxisPlacement) boreTool(t *testing.T, doc *decad.Document, radius float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(p.world(r3.Vec{X: 4}), p.world(r3.Vec{Y: 1}), p.world(r3.Vec{Z: 1}))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	return tool
}

// boreCut is a joined body about one axis cut by the coaxial bore, with the
// cut's volume and area.
type boreCut struct {
	cut    *decad.Body
	volume decad.Measurement
	area   decad.Measurement
}

// cutBore cuts u's joined body with the placement's bore tool of the given
// radius through the public Cut. The result passes VerifyAll, is watertight
// with Euler characteristic 0, holds less volume than the joined body, keeps
// the tool's side origins on a cylindrical face and keeps keptCaps Revolve
// caps' origins on planar faces: both for an annular bore, and only the high
// cap for a bore through the band, which removes the low cap.
func (p revolveAxisPlacement) cutBore(t *testing.T, u revolveAxisUnion, radius float64, keptCaps int) boreCut {
	t.Helper()
	tool := p.boreTool(t, u.doc, radius)
	cut, err := decad.Cut(t.Context(), u.joined, tool)
	require.NoError(t, err)
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 0, revolveLugEuler(mesh))
	requireClosedMesh(t, mesh)
	volume, err := cut.Volume()
	require.NoError(t, err)
	area, err := cut.Area()
	require.NoError(t, err)
	require.Greater(t, u.volume.Value.Base()-u.volume.Bound.Base(), volume.Value.Base()+volume.Bound.Base())
	keeps := func(origins []decad.FeatureRef, kind func(decad.Surface) bool) bool {
		for _, face := range cut.Faces() {
			if kind(face.Surface()) && slices.Equal(origins, face.Origins()) {
				return true
			}
		}
		return false
	}
	planar := func(s decad.Surface) bool { _, ok := s.(decad.Plane); return ok }
	cylindrical := func(s decad.Surface) bool { _, ok := s.(decad.Cylinder); return ok }
	caps, kept := 0, 0
	for _, face := range u.hub.Faces() {
		if planar(face.Surface()) {
			caps++
			if keeps(face.Origins(), planar) {
				kept++
			}
		}
	}
	require.Equal(t, 2, caps)
	require.Equal(t, keptCaps, kept, "the cut keeps the wrong number of Revolve caps")
	walls := 0
	for _, face := range tool.Faces() {
		if cylindrical(face.Surface()) {
			walls++
			require.True(t, keeps(face.Origins(), cylindrical), "the bore wall lost its source origins")
		}
	}
	require.Equal(t, 1, walls)
	return boreCut{cut: cut, volume: volume, area: area}
}

// generalBoreAxes are the axes the coaxial bore Cut tests run about, each
// with the input slack of TestUnionHubAndRadialLugsAboutTiltedAxis: none for
// (0, 1, 0) and for the XZ-plane blank about X, both exact quarter-turn
// images of +X, and the rounding of the tilted fixtures about (1, 2, 0) and
// a 70-degree shaft direction rounded to 26 significant bits.
func generalBoreAxes(t *testing.T) []struct {
	name      string
	placement revolveAxisPlacement
	shift     float64
} {
	t.Helper()
	shaft := 70 * math.Pi / 180
	return []struct {
		name      string
		placement revolveAxisPlacement
		shift     float64
	}{
		{"(0,1,0)", permutedAxisPlacement(t), 0},
		{"XZ plane about X", xzAxisPlacement(t), 0},
		{"(1,2,0)", tiltedAxisPlacement(t, r3.Vec{X: 1, Y: 2}, 0x1p-30), 1e-8},
		{"26-bit 70 degrees",
			tiltedAxisPlacement(t, r3.Vec{X: round26(math.Cos(shaft)), Y: round26(math.Sin(shaft))}, 0x1p-22), 1e-6},
	}
}

// requireBoreAboutGeneralAxes cuts the tapered hub and three copied lugs with
// a bore of the given radius about +X and about each general axis, and
// checks each general result's volume against the +X one within the proven
// bounds and the axis's input slack.
func requireBoreAboutGeneralAxes(t *testing.T, radius float64, keptCaps int) {
	t.Helper()
	turns := []float64{120, 240}
	x := xAxisPlacement(t)
	control := x.cutBore(t, x.union(t, taperedHubOutline, turns, false), radius, keptCaps)
	for _, tc := range generalBoreAxes(t) {
		t.Run(tc.name, func(t *testing.T) {
			general := tc.placement.cutBore(t, tc.placement.union(t, taperedHubOutline, turns, false), radius, keptCaps)
			requireVolumesAgree(t, control.volume, general.volume, inputSlack(general.area, tc.shift))
			t.Logf("+X bored volume %.12g ± %.3g, bored volume %.12g ± %.3g", control.volume.Value.Base(),
				control.volume.Bound.Base(), general.volume.Value.Base(), general.volume.Bound.Base())
		})
	}
}

// The coaxial bore Cut of a tapered hub and three copied lugs about a general
// axis takes the bore route at the default tolerance
// (agent-docs/revolve-axis-design.md §9). A radius-1 bore lies inside both
// cap radii. The result is watertight with Euler characteristic 0, passes
// VerifyAll, keeps both caps, and its volume agrees with the +X bore within
// the proven bounds.
func TestRevolveSectorCylinderCutAboutGeneralAxis(t *testing.T) {
	requireBoreAboutGeneralAxes(t, 1, 2)
}

// A radius-2 bore lies between the low cap's radius 1.5 and the high cap's
// radius 3, so it crosses the straight band from (5, 2.5) to (6, 1.5) and
// removes the low cap (agent-docs/revolve-axis-design.md §9.7). About each
// general axis the result is watertight with Euler characteristic 0, passes
// VerifyAll, keeps only the high cap, and its volume agrees with the +X band
// bore within the proven bounds.
func TestRevolveSectorCylinderBandCutAboutGeneralAxis(t *testing.T) {
	requireBoreAboutGeneralAxes(t, 2, 1)
}
