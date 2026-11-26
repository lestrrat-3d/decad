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

// arcToothShape sizes a tooth-style lug whose root is a filleted gap: a short
// concave arc on each flank, a straight run-out line tangent to it that leaves
// the hub's root cone at a small angle, and a shallow root arc inside the hub.
// All lengths are in the sketch plane at X = 10, where the hub's root cone has
// radius 5.
type arcToothShape struct {
	half     float64 // half width of the straight flanks
	fillet   float64 // fillet arc radius
	runout   float64 // run-out crossing angle, radians
	rootDrop float64 // depth of the root arc below the root cone
}

// arcToothPoints holds the fixed construction points of one tooth side as
// (x, y) in the sketch plane, y > 0. The other side mirrors it.
type arcToothPoints struct {
	flankStart, center, runoutStart, runoutEnd [2]float64
}

// arcToothSide derives one side's points. The fillet circle touches the flank
// line y = half at flankStart and the circle of radius rootRadius - h about the
// origin; the run-out leaves the circle tangentially runout before that
// touching point and ends on the root arc.
func arcToothSide(shape arcToothShape, rootRadius float64) arcToothPoints {
	h := shape.fillet * (1 - math.Cos(shape.runout)) / 2
	reach := rootRadius - h + shape.fillet // origin to fillet centre
	offset := shape.half + shape.fillet
	cx := -math.Sqrt(reach*reach - offset*offset)
	// Angle of the direction from the centre toward the origin, then back off
	// by the run-out angle: the arc is walked counter-clockwise from -90 deg.
	toward := math.Atan2(-offset, -cx)
	end := toward - shape.runout
	t3 := [2]float64{cx + shape.fillet*math.Cos(end), offset + shape.fillet*math.Sin(end)}
	dir := [2]float64{-math.Sin(end), math.Cos(end)}
	// Solve |t3 + s*dir| = rootRadius - rootDrop for the entry point, the smaller root.
	rootArc := rootRadius - shape.rootDrop
	b := t3[0]*dir[0] + t3[1]*dir[1]
	c := t3[0]*t3[0] + t3[1]*t3[1] - rootArc*rootArc
	s := -b - math.Sqrt(b*b-c)
	return arcToothPoints{
		flankStart:  [2]float64{cx, shape.half},
		center:      [2]float64{cx, offset},
		runoutStart: t3,
		runoutEnd:   [2]float64{t3[0] + s*dir[0], t3[1] + s*dir[1]},
	}
}

// arcToothHubAndLug builds a revolved hub and one trimmed point-Loft tooth
// whose sketch has concave fillet arcs away from the hub's axis, run-out lines
// that cross the root cone, and a root arc below it.
func arcToothHubAndLug(t testing.TB, shape arcToothShape) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = section.CreatePoint(point.U, point.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	hub, err := decad.Revolve(doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{}).Do(t.Context())
	require.NoError(t, err)

	lugWorld := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	plane, err := lugWorld.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	lug, err := lugWorld.CreateSketch(plane)
	require.NoError(t, err)
	side := arcToothSide(shape, 5)
	point := func(p [2]float64, sign float64) *sketch.Point {
		created := lug.CreatePoint(p[0], sign*p[1])
		lug.Fix(created)
		return created
	}
	center := point([2]float64{}, 1)
	flankLow, flankHigh := point(side.flankStart, -1), point(side.flankStart, 1)
	filletLow, filletHigh := point(side.center, -1), point(side.center, 1)
	runoutLowStart, runoutHighStart := point(side.runoutStart, -1), point(side.runoutStart, 1)
	runoutLowEnd, runoutHighEnd := point(side.runoutEnd, -1), point(side.runoutEnd, 1)
	tipLow := point([2]float64{-6, shape.half}, -1)
	tipHigh := point([2]float64{-6, shape.half}, 1)

	lug.CreateArc(filletLow, runoutLowStart, flankLow)
	lug.CreateLine(flankLow, tipLow)
	lug.CreateArc(center, tipHigh, tipLow)
	lug.CreateLine(tipHigh, flankHigh)
	lug.CreateArc(filletHigh, flankHigh, runoutHighStart)
	lug.CreateLine(runoutHighStart, runoutHighEnd)
	lug.CreateArc(center, runoutHighEnd, runoutLowEnd)
	lug.CreateLine(runoutLowEnd, runoutLowStart)
	_, err = lug.Solve(t.Context())
	require.NoError(t, err)
	lugProfiles := lug.Profiles()
	require.Len(t, lugProfiles, 1)
	tooth, err := decad.LoftFromPoint(doc, r3.Vec{}, lug, lugProfiles[0]).Do(t.Context())
	require.NoError(t, err)
	toeTrim, err := decad.Cut(tooth, pointConeTool(t, doc, 7.5)).Do(t.Context())
	require.NoError(t, err)
	trimmed, err := decad.Intersect(toeTrim, pointConeTool(t, doc, 12)).Do(t.Context())
	require.NoError(t, err)
	return doc, hub, trimmed
}

// arcToothRing joins the hub with four copies of the tooth, a quarter turn
// apart about the hub axis.
func arcToothRing(t testing.TB, hub, tooth *decad.Body) *decad.Body {
	t.Helper()
	teeth := []*decad.Body{tooth}
	for i := 1; i < 4; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(math.Pi/2*float64(i)))
		require.NoError(t, err)
		copied, err := decad.Copy(t.Context(), tooth, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		teeth = append(teeth, copied)
	}
	joined, err := decad.Union(append([]*decad.Body{hub}, teeth...)...).Do(t.Context())
	require.NoError(t, err)
	return joined
}

var arcToothDefault = arcToothShape{half: 1.3, fillet: 0.6, runout: 20 * math.Pi / 180, rootDrop: 0.1}

// TestUnionArcToothSectorJoin joins a hub with a tooth whose fillet arcs and
// shallow root arc are general arcs, not the single short arc about the
// tooth's own axis. Sketch proves each arc's side of the root cone, and the
// sector join builds, tessellates verified and refines.
func TestUnionArcToothSectorJoin(t *testing.T) {
	doc, hub, tooth := arcToothHubAndLug(t, arcToothDefault)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	joined := arcToothRing(t, hub, tooth)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
	require.Equal(t, 2, revolveLugEuler(mesh))
	require.Contains(t, doc.Bodies(), joined)

	refined, err := decad.Refine(joined, units.Millimeters(0.05)).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, refined), 0.05)
	_, err = refined.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
}

// TestUnionArcToothSectorJoinRefusals keeps an unproven shape refused: a root
// arc so close to the cone that Sketch's side margin is below the join's cost.
func TestUnionArcToothSectorJoinRefusals(t *testing.T) {
	for _, test := range []struct {
		name  string
		shape arcToothShape
	}{
		{"root arc at the cone", arcToothShape{half: 1.3, fillet: 0.6, runout: 20 * math.Pi / 180, rootDrop: 0.0005}},
	} {
		t.Run(test.name, func(t *testing.T) {
			doc, hub, tooth := arcToothHubAndLug(t, test.shape)
			before := doc.Bodies()
			_, err := decad.Union(hub, tooth).Do(t.Context())
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, before, doc.Bodies())
		})
	}
}
