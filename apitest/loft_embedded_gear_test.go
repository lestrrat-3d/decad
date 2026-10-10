package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type embeddedGearCase struct {
	teeth, samples int
	pressure       float64
	fillet         float64
	bore           float64
	chamfer        float64
}

func (c embeddedGearCase) dimensions() (base, root, tip float64) {
	const module = 0.5
	pitch := module * float64(c.teeth) / 2
	return pitch * math.Cos(c.pressure),
		(module*float64(c.teeth) - 2.5*module) / 2,
		(module*float64(c.teeth) + 2*module) / 2
}

// embeddedGearProfile uses the gallery's original fit points and full root
// circle. Sketch's union selects all bounded regions and cancels their shared
// intervals, leaving one gear outline with the original spline fragments.
func embeddedGearProfile(t *testing.T, world *sketch.World, plane *sketch.Plane,
	c embeddedGearCase) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	base, root, tip := c.dimensions()
	const module = 0.5
	pitch := module * float64(c.teeth) / 2
	atPitch := gearInvolute(base, pitch)
	rotation := math.Pi/(2*float64(c.teeth)) - math.Atan2(-atPitch.y, atPitch.x)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	for tooth := range c.teeth {
		angle := 2 * math.Pi * float64(tooth) / float64(c.teeth)
		left, right := make([]*sketch.Point, c.samples), make([]*sketch.Point, c.samples)
		for i := range c.samples {
			radius := base + (tip-base)*float64(i)/float64(c.samples-1)
			point := gearInvolute(base, radius)
			point.y = -point.y
			point = gearRotate(point, rotation)
			leftXY := gearRotate(point, angle)
			point.y = -point.y
			rightXY := gearRotate(point, angle)
			left[i] = s.CreatePoint(leftXY.x, leftXY.y)
			right[i] = s.CreatePoint(rightXY.x, rightXY.y)
			s.Fix(left[i])
			s.Fix(right[i])
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(center, right[c.samples-1], left[c.samples-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
	}
	s.CreateCircle(center, root)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	regions := s.Profiles()
	require.GreaterOrEqual(t, len(regions), c.teeth+1)
	indices := make([]int, len(regions))
	for i := range indices {
		indices[i] = i
		require.True(t, regions[i].Valid)
	}
	profile, err := s.UnionProfiles(indices...)
	require.NoError(t, err)
	require.True(t, profile.Valid)
	require.Empty(t, profile.Holes)
	require.Len(t, profile.Outer, 4*c.teeth)
	kinds := map[string]int{}
	for _, edge := range profile.Outer {
		if edge.Partial {
			require.True(t, edge.TExact, "every exposed trim must be certified by Sketch")
		}
		switch curve := edge.Entity.(type) {
		case *sketch.FitSpline:
			kinds["flank"]++
			require.True(t, edge.Partial)
			require.Greater(t, edge.TStart, 0.0)
			require.InDelta(t, 1, edge.TEnd, 1e-14)
			x, y := curve.Eval(edge.TStart)
			require.InDelta(t, root, math.Hypot(x, y), 1e-9)
			x, y = curve.Eval(edge.TEnd)
			require.InDelta(t, tip, math.Hypot(x, y), 1e-9)
		case *sketch.Arc:
			kinds["tip"]++
		case *sketch.Circle:
			kinds["root"]++
		default:
			t.Fatalf("unexpected gear boundary curve %T", edge.Entity)
		}
	}
	require.Equal(t, 2*c.teeth, kinds["flank"])
	require.Equal(t, c.teeth, kinds["tip"])
	require.Equal(t, c.teeth, kinds["root"])
	return s, profile
}

func embeddedGearLoft(t *testing.T, c embeddedGearCase) (*decad.Document, *decad.Body, []r3.Vec) {
	t.Helper()
	const thickness = 2.0
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, thickness), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s0, p0 := embeddedGearProfile(t, world, world.XY(), c)
	s1, p1 := embeddedGearProfile(t, world, upper, c)
	// The real Sketch trim endpoints identify the root junctions. At a fine
	// chord target the loft also holds stations inside each root-circle piece.
	rootCorners := make([]r3.Vec, 0, 2*c.teeth)
	for _, edge := range p0.Outer {
		if fit, ok := edge.Entity.(*sketch.FitSpline); ok {
			x, y := fit.Eval(edge.TStart)
			rootCorners = append(rootCorners, r3.NewVec(x, y, 0))
		}
	}
	require.Len(t, rootCorners, 2*c.teeth)
	doc := decad.New()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, body.Lumps(), 1)
	box, err := body.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 0, box.Min.Z, box.Bound.Base()+1e-8)
	require.InDelta(t, thickness, box.Max.Z, box.Bound.Base()+1e-8)
	_, _, tip := c.dimensions()
	require.LessOrEqual(t, math.Max(math.Abs(box.Min.X), math.Abs(box.Max.X)), tip+box.Bound.Base()+1e-8)
	require.LessOrEqual(t, math.Max(math.Abs(box.Min.Y), math.Abs(box.Max.Y)), tip+box.Bound.Base()+1e-8)
	tipVertices := 0
	for _, vertex := range body.Vertices() {
		point := vertex.Position().Value
		if math.Abs(math.Hypot(point.X, point.Y)-tip) < 1e-8 {
			tipVertices++
		}
	}
	require.GreaterOrEqual(t, tipVertices, 2*c.teeth)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, p0.Area*thickness, volume.Value.Base(), volume.Bound.Base()+0.02)
	return doc, body, rootCorners
}

func embeddedGearFillet(t *testing.T, body *decad.Body, c embeddedGearCase, rootCorners []r3.Vec) *decad.Body {
	t.Helper()
	_, root, _ := c.dimensions()
	axis := r3.NewVec(0, 0, 1)
	axial, err := decad.Edges(decad.ParallelTo(axis)).SelectEdges(body)
	require.NoError(t, err)
	var corners []r3.Vec
	rootEdges := 0
	for _, edge := range axial {
		point := edge.Start().Position().Value
		if math.Abs(math.Hypot(point.X, point.Y)-root) < 1e-8 {
			rootEdges++
			for _, expected := range rootCorners {
				if math.Hypot(point.X-expected.X, point.Y-expected.Y) < 1e-8 {
					corners = append(corners, point)
					break
				}
			}
		}
	}
	require.Len(t, corners, 2*c.teeth)
	if c.teeth == 60 {
		require.Equal(t, c.teeth, rootEdges-len(corners), "interior root-circle stations are not corners")
	}
	query := decad.Edges(decad.ParallelTo(axis), decad.EndpointAt(corners[0]))
	for _, corner := range corners[1:] {
		query.Or(decad.ParallelTo(axis), decad.EndpointAt(corner))
	}
	rounded, err := body.Fillet(t.Context(), query.Exactly(2*c.teeth), units.Millimeters(c.fillet))
	require.NoError(t, err)
	require.True(t, rounded.IsSolid())
	require.Len(t, rounded.Lumps(), 1)
	roles := map[string]struct{}{}
	for _, face := range rounded.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(0,") {
				roles[origin.Role] = struct{}{}
			}
		}
	}
	require.Len(t, roles, 2*c.teeth)
	return rounded
}

func embeddedGearBore(t *testing.T, doc *decad.Document, body *decad.Body, c embeddedGearCase) *decad.Body {
	t.Helper()
	const thickness, margin = 2.0, 1.0
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, -margin), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, c.bore/2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tool, err := doc.Extrude(s, profiles[0], decad.Distance{
		D: units.Millimeters(thickness + 2*margin), Dir: decad.Along,
	})
	require.NoError(t, err)
	before, err := body.Volume()
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), body, tool)
	require.NoError(t, err)
	require.True(t, bored.IsSolid())
	require.Len(t, bored.Lumps(), 1)
	after, err := bored.Volume()
	require.NoError(t, err)
	want := math.Pi * math.Pow(c.bore/2, 2) * thickness
	require.InDelta(t, want, before.Value.Base()-after.Value.Base(),
		before.Bound.Base()+after.Bound.Base()+0.02)
	return bored
}

func embeddedGearChamfer(t *testing.T, body *decad.Body, c embeddedGearCase, bore bool) *decad.Body {
	t.Helper()
	selection := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).
		Or(decad.OuterLoopOf(decad.CapEnd(body)))
	chamfered, err := body.Chamfer(t.Context(), selection, units.Millimeters(c.chamfer))
	require.NoError(t, err)
	require.True(t, chamfered.IsSolid())
	require.Len(t, chamfered.Lumps(), 1)
	chamferFaces := 0
	for _, face := range chamfered.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "chamferCap(") {
				chamferFaces++
				break
			}
		}
	}
	require.Positive(t, chamferFaces)
	for _, cap := range []decad.FeatureRef{decad.CapStart(chamfered), decad.CapEnd(chamfered)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(cap)).Exactly(1).SelectFaces(chamfered)
		require.NoError(t, err)
		if bore {
			require.Len(t, faces[0].Loops(), 2)
			require.Len(t, faces[0].Loops()[1].Edges(), 1)
			circle, ok := faces[0].Loops()[1].Edges()[0].Curve().(decad.Circle3)
			require.True(t, ok, "the bore rim stays a sharp circle")
			require.InDelta(t, c.bore/2, circle.Radius.Base(), 1e-10)
		} else {
			require.Len(t, faces[0].Loops(), 1)
		}
	}
	return chamfered
}

func runEmbeddedGearFeatures(t *testing.T, c embeddedGearCase) {
	t.Helper()
	doc, lofted, rootCorners := embeddedGearLoft(t, c)
	rounded := embeddedGearFillet(t, lofted, c, rootCorners)
	noBore, err := rounded.Duplicate(t.Context())
	require.NoError(t, err)
	noBore = embeddedGearChamfer(t, noBore, c, false)
	bored := embeddedGearBore(t, doc, rounded, c)
	withBore := embeddedGearChamfer(t, bored, c, true)
	require.ElementsMatch(t, []*decad.Body{noBore, withBore}, doc.Bodies())
	for _, result := range []*decad.Body{noBore, withBore} {
		mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1),
			decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
		require.False(t, math.IsInf(mesh.Bound().Base(), 0))
		require.False(t, math.IsNaN(mesh.Bound().Base()))
	}
}

func TestLoftEmbeddedThirtyToothGearFeatures(t *testing.T) {
	runEmbeddedGearFeatures(t, embeddedGearCase{
		teeth: 30, samples: 5, pressure: 30 * math.Pi / 180,
		fillet: 0.05, bore: 2, chamfer: 0.02,
	})
}

func TestLoftEmbeddedSixtyToothGearFeatures(t *testing.T) {
	const teeth = 60
	const module = 0.5
	pressure := 20 * math.Pi / 180
	root := (module*teeth - 2.5*module) / 2
	fillet := 0.9 * root * (math.Pi/teeth - 2*(math.Tan(pressure)-pressure)) / 2
	require.Positive(t, fillet)
	runEmbeddedGearFeatures(t, embeddedGearCase{
		teeth: teeth, samples: 15, pressure: pressure,
		fillet: fillet, bore: 3, chamfer: 0.02,
	})
}
