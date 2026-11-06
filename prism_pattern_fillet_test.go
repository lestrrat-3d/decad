package decad

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternFilletOfCopiedExtrudes(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, prismPatternInsetBlank)
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	selection := prismPatternRootSelection(t, joined)
	rounded, err := Fillet(t.Context(), joined, selection, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	after, err := rounded.Volume()
	require.NoError(t, err)
	require.Greater(t, after.Value.Base(), before.Value.Base())
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestPrismPatternFilletAcrossCircularWallStations(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, prismPatternInsetBlank)
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	selection := prismPatternRootSelection(t, joined)
	edges, err := selection.SelectEdges(joined)
	require.NoError(t, err)
	contacts, err := prismpattern.FilletContacts(t.Context(), joined.n.Payload.(facetedPayload), edgeNodes(edges), 0.14)
	require.NoError(t, err)
	// Selecting one root twice is refused rather than rounded twice.
	_, err = prismpattern.FilletContacts(t.Context(), joined.n.Payload.(facetedPayload),
		edgeNodes(append([]*Edge{edges[0]}, edges...)), 0.14)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "selects one root twice")
	multiStation := false
	for _, contact := range contacts {
		multiStation = multiStation || len(contact.RingCut.Positions) > 2
	}
	require.True(t, multiStation)
	rounded, err := Fillet(t.Context(), joined, selection,
		units.Millimeters(0.14))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestPrismPatternFilletSectorBeyondRootDisk(t *testing.T) {
	doc, blank, section, rotated := prismPatternNarrowTabBodies(t)
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	// The old 3r disk reaches the fitted flank. The analytic arc and its
	// connector cutback stay inside a narrower radial and angular envelope.
	const radius = 0.055
	require.Greater(t, 3+3*radius, 3.15)
	require.Greater(t, 2*3*math.Sin(0.14), 6*radius)
	rounded, err := Fillet(t.Context(), joined, prismPatternRootSelection(t, joined),
		units.Millimeters(radius))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func prismPatternNarrowTabBodies(t *testing.T) (*Document, *Body, *Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	point := func(radius, angle float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
		s.Fix(p)
		return p
	}
	rootRight, rootLeft := point(3, -0.14), point(3, 0.14)
	right := []*sketch.Point{point(3.15, -0.14), point(3.55, -0.137),
		point(3.75, -0.13), point(4, -0.12)}
	left := []*sketch.Point{point(3.15, 0.14), point(3.55, 0.137),
		point(3.75, 0.13), point(4, 0.12)}
	s.CreateLine(rootRight, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rootLeft)
	s.CreateArc(center, rootRight, rootLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := New()
	section, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	blankSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	c := blankSketch.CreatePoint(0, 0)
	blankSketch.Fix(c)
	blankSketch.CreateCircle(c, prismPatternInsetBlank)
	_, err = blankSketch.Solve(t.Context())
	require.NoError(t, err)
	blank, err := Extrude(t.Context(), doc, blankSketch, blankSketch.Profiles()[0],
		Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	rotated, err := Copy(t.Context(), section, WithCopyPlacement(turn))
	require.NoError(t, err)
	return doc, blank, section, rotated
}

// A gear-like tooth whose radial root line is shorter than 2r: the complete
// fillet circle reaches past the line's far end and into the flank, but each
// changed corner stays inside its line foot's radius. Every root corner of
// the copied pattern rounds, and the volume gain is the analytic corner area.
func TestPrismPatternFilletPastShortRadialLine(t *testing.T) {
	type gear struct {
		teeth                 int
		root, base, tip, half float64
		radius, inset         float64
	}
	// A module-1, 20° involute gear: the root, base and tip radii, the
	// tooth's half angle at its root lines and a root fillet of 0.9 of half
	// the root gap. The root arc lies inset inside the blank.
	involute := func(teeth int, inset float64) gear {
		n := float64(teeth)
		pressure := 20 * math.Pi / 180
		inv := math.Tan(pressure) - pressure
		root := (n - 2.5) / 2
		return gear{teeth: teeth, root: root, base: n / 2 * math.Cos(pressure), tip: (n + 2) / 2,
			half: math.Pi/(2*n) + inv, radius: 0.9 * root * (math.Pi/n - 2*inv) / 2, inset: inset}
	}
	// The 1 nm inset leaves the union of the sections no region it can place,
	// so the copied-prism route joins. That route needs every PatternCopies
	// placement to be an exact axial rotation, which these tooth counts give.
	// At 22 teeth the line foot lands 0.1 mm short of the line's far end.
	for _, tc := range []gear{involute(12, 1e-9), involute(17, 1e-9), involute(22, 1e-9)} {
		t.Run(fmt.Sprintf("%d teeth", tc.teeth), func(t *testing.T) {
			// The fillet circle overruns the radial line, yet its line foot
			// (for a radial line, at sqrt((R+r)²-r²)) lands on it.
			foot := math.Sqrt((tc.root+tc.radius)*(tc.root+tc.radius) - tc.radius*tc.radius)
			require.Greater(t, tc.root+2*tc.radius, tc.base)
			require.Less(t, foot, tc.base)

			doc, blank, tooth := prismPatternGearToothBodies(t, tc.teeth, tc.root, tc.base, tc.tip,
				tc.half, tc.inset, 2)
			copies, err := PatternCopies(t.Context(), tooth, CircularPattern{Axis: r3.Vec{Z: 1}, Count: tc.teeth})
			require.NoError(t, err)
			joined, err := Union(t.Context(), blank, append([]*Body{tooth}, copies...)...)
			require.NoError(t, err)
			require.NotNil(t, joined.n.Payload.(facetedPayload).PrismPatternJoin, "the copied-prism route joins")
			before, err := joined.Volume()
			require.NoError(t, err)

			rootRadius := tc.root - tc.inset
			axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
			require.NoError(t, err)
			var selection *EdgeQuery
			roots := 0
			for _, edge := range axial {
				p := edge.Start().Position().Value
				if math.Abs(math.Hypot(p.X, p.Y)-rootRadius) > tc.inset/10 {
					continue
				}
				roots++
				if selection == nil {
					selection = Edges(ParallelTo(r3.Vec{Z: 1}), EndpointAt(p))
					continue
				}
				selection.Or(ParallelTo(r3.Vec{Z: 1}), EndpointAt(p))
			}
			require.Equal(t, 2*tc.teeth, roots)

			rounded, err := Fillet(t.Context(), joined, selection.Exactly(roots), units.Millimeters(tc.radius))
			require.NoError(t, err)
			require.Len(t, doc.Bodies(), 1)
			require.True(t, rounded.IsSolid())
			mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Less(t, mesh.Bound().Base(), 0.1)

			// A radial line meeting the circle of radius R at a right angle:
			// triangle (axis, line foot, fillet center) less the fillet's
			// sector at its center and the blank's sector to the circle foot.
			R, r := tc.root, tc.radius
			corner := r*foot/2 - r*r*math.Acos(r/(R+r))/2 - R*R*math.Atan2(r, foot)/2
			after, err := rounded.Volume()
			require.NoError(t, err)
			gain := after.Value.Base() - before.Value.Base()
			want := float64(2*tc.teeth) * corner * 2
			require.InDelta(t, want, gain, 0.02*want)
		})
	}
}

// prismPatternGearToothBodies draws one tooth of teeth with radial root lines
// from inset inside the root circle to the base circle, fitted involute
// flanks, and a tip arc, and a blank on the root circle, both extruded height.
func prismPatternGearToothBodies(t *testing.T, teeth int, root, base, tip, half, inset, height float64) (*Document, *Body, *Body) {
	t.Helper()
	require.Less(t, half, math.Pi/float64(teeth))
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	point := func(radius, angle float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
		s.Fix(p)
		return p
	}
	rootRight, rootLeft := point(root-inset, -half), point(root-inset, half)
	const samples = 8
	right := make([]*sketch.Point, samples)
	left := make([]*sketch.Point, samples)
	for i := range samples {
		// Each flank is an involute of the base circle, leaving it at the
		// root line's angle and leaning in to the tip.
		radius := base + (tip-base)*float64(i)/(samples-1)
		pressure := math.Acos(base / radius)
		angle := half - (math.Tan(pressure) - pressure)
		right[i], left[i] = point(radius, -angle), point(radius, angle)
	}
	s.CreateLine(rootRight, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[samples-1], left[samples-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rootLeft)
	s.CreateArc(center, rootRight, rootLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.Len(t, s.Profiles()[0].Outer, 6)
	doc := New()
	tooth, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	blankSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	c := blankSketch.CreatePoint(0, 0)
	blankSketch.Fix(c)
	blankSketch.CreateCircle(c, root)
	_, err = blankSketch.Solve(t.Context())
	require.NoError(t, err)
	blank, err := Extrude(t.Context(), doc, blankSketch, blankSketch.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return doc, blank, tooth
}

func TestPrismPatternFilletRefusesUnprovedCornerAndLargeRadius(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, prismPatternInsetBlank)
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var exterior *Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Hypot(p.X, p.Y) > 3.3 {
			exterior = edge
			break
		}
	}
	require.NotNil(t, exterior)
	_, err = Fillet(t.Context(), joined, Edges(ParallelTo(r3.Vec{Z: 1}),
		EndpointAt(exterior.Start().Position().Value)), units.Millimeters(0.05))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
	// The tab's radial root lines run from 3 to 3.35. At r = 0.4 the line
	// foot, sqrt(9+6r) = 3.38 from the axis, lies past the line's far end.
	_, err = Fillet(t.Context(), joined, prismPatternRootSelection(t, joined), units.Millimeters(0.4))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "contact escapes its connector")
	require.Len(t, doc.Bodies(), 1)
}

// rimToothCase is an involute tooth whose root arc lies on the blank's rim,
// thinned at its root lines by thin radians on each side as backlash thins a
// tooth.
type rimToothCase struct {
	teeth int
	thin  float64
}

// rimToothFillet draws the tc tooth and its blank on a module-1, 20° involute
// gear, extruded height, and joins the tooth's pattern. It rounds all 2N root
// corners with a radius of 0.9 of half the root gap and returns the rounded
// body, and whether the union of the sections made the join.
func rimToothFillet(t *testing.T, tc rimToothCase, height float64) (*Body, bool) {
	t.Helper()
	n := float64(tc.teeth)
	pressure := 20 * math.Pi / 180
	inv := math.Tan(pressure) - pressure
	root := (n - 2.5) / 2
	half := math.Pi/(2*n) + inv - tc.thin
	radius := 0.9 * root * (math.Pi/n - 2*(half-math.Pi/(2*n))) / 2
	doc, blank, tooth := prismPatternGearToothBodies(t, tc.teeth, root, n/2*math.Cos(pressure), (n+2)/2, half, 0, height)
	copies, err := PatternCopies(t.Context(), tooth, CircularPattern{Axis: r3.Vec{Z: 1}, Count: tc.teeth})
	require.NoError(t, err)
	joined, err := Union(t.Context(), blank, append([]*Body{tooth}, copies...)...)
	require.NoError(t, err)
	_, bySections := joined.n.Payload.(loftPayload)

	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var corners []r3.Vec
	for _, edge := range axial {
		p := edge.Start().Position().Value
		for i := range tc.teeth {
			step := 2 * math.Pi * float64(i) / float64(tc.teeth)
			for _, angle := range []float64{step - half, step + half} {
				if math.Hypot(p.X-root*math.Cos(angle), p.Y-root*math.Sin(angle)) <= 1e-6 {
					corners = append(corners, p)
				}
			}
		}
	}
	require.Len(t, corners, 2*tc.teeth)
	selection := Edges(ParallelTo(r3.Vec{Z: 1}), EndpointAt(corners[0]))
	for _, corner := range corners[1:] {
		selection.Or(ParallelTo(r3.Vec{Z: 1}), EndpointAt(corner))
	}
	rounded, err := Fillet(t.Context(), joined, selection.Exactly(len(corners)), units.Millimeters(radius))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, rounded.IsSolid())
	fillets := map[string]struct{}{}
	for _, face := range rounded.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(") {
				fillets[origin.Role] = struct{}{}
			}
		}
	}
	require.Len(t, fillets, 2*tc.teeth)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return rounded, bySections
}

// A tooth pattern whose root arcs lie on the blank's rim joins through the
// union of the sections and takes its root fillets, whatever the tooth count
// and width. On amd64 each case here has copies whose root arcs all round
// strictly inside the blank, which the copied-prism route also admits, so a
// route chosen by that rounding would pick it. At 27 teeth the fillet's
// contact escapes the short radial root line onto the involute flank, which
// only the union of the sections supports.
func TestRimToothPatternJoinsBySectionUnion(t *testing.T) {
	for _, tc := range []rimToothCase{{8, 0.005}, {11, 0.004}, {17, 0.003}, {18, 0.006}, {27, 0.0016}} {
		t.Run(fmt.Sprintf("%d teeth thinned %v", tc.teeth, tc.thin), func(t *testing.T) {
			_, bySections := rimToothFillet(t, tc, 2)
			require.True(t, bySections, "the union of the sections joins")
		})
	}
}

// The rounded rim-tooth pattern reads its volume within half a cubic
// millimetre at 10 mm thick.
func TestRimToothPatternFilletVolumeBound(t *testing.T) {
	rounded, _ := rimToothFillet(t, rimToothCase{17, 0.003}, 10)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	require.Less(t, volume.Bound.Base(), 0.5)
}

func prismPatternRootSelection(t *testing.T, joined *Body) *EdgeQuery {
	t.Helper()
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var roots []*Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Abs(math.Hypot(p.X, p.Y)-3) < 1e-8 {
			roots = append(roots, edge)
		}
	}
	require.Len(t, roots, 4)
	selection := Edges(ParallelTo(r3.Vec{Z: 1}), EndpointAt(roots[0].Start().Position().Value))
	for _, edge := range roots[1:] {
		selection.Or(ParallelTo(r3.Vec{Z: 1}), EndpointAt(edge.Start().Position().Value))
	}
	return selection.Exactly(len(roots))
}
