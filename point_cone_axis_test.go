package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The general-axis point-cone trim fixtures (agent-docs/revolve-axis-design.md
// §7.2). The +X fixture lofts pointLoftTooth's profile from the origin to the
// plane X = 8 and trims it with cones revolved from the world XY sketch about
// U. The permuted fixture maps every input by the exact rotation
// (x, y, z) → (z, x, y), which takes world X to world Y, so its trims are about
// A = (0, 1, 0). The tilted fixture trims about A = (1, 2, 0), whose length √5
// is irrational.

// axisPlane returns a world plane with the given frame.
func axisPlane(t *testing.T, world *sketch.World, origin, u, v r3.Vec) *sketch.Plane {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	return plane
}

// axisTooth lofts pointLoftTooth's profile, drawn on a plane with the given
// frame, from the world origin.
func axisTooth(t *testing.T, doc *Document, origin, u, v r3.Vec) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(axisPlane(t, world, origin, u, v))
	require.NoError(t, err)
	points := []*sketch.Point{
		s.CreatePoint(3, -1), s.CreatePoint(4, -1.5), s.CreatePoint(5, -1),
		s.CreatePoint(5, 1), s.CreatePoint(3, 1),
	}
	for _, point := range points {
		s.Fix(point)
	}
	_, err = s.CreateFitSpline(points[0], points[1], points[2])
	require.NoError(t, err)
	for i := 2; i < len(points); i++ {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return body
}

// axisConeTool revolves the triangle far, rim, apex, drawn on a plane through
// the origin with axes u and v, about the sketch line from (0, 0) to end.
func axisConeTool(t *testing.T, doc *Document, u, v r3.Vec, end planePoint, far, rim, apex planePoint) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(axisPlane(t, world, r3.Vec{}, u, v))
	require.NoError(t, err)
	points := []*sketch.Point{s.CreatePoint(far.U, far.V), s.CreatePoint(rim.U, rim.V),
		s.CreatePoint(apex.U, apex.V)}
	for _, point := range points {
		s.Fix(point)
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: end}, FullRevolution{})
	require.NoError(t, err)
	return tool
}

var (
	worldX = r3.Vec{X: 1}
	worldY = r3.Vec{Y: 1}
	worldZ = r3.Vec{Z: 1}
)

// permutedTooth is pointLoftTooth mapped by (x, y, z) → (z, x, y).
func permutedTooth(t *testing.T, doc *Document) *Body {
	t.Helper()
	return axisTooth(t, doc, r3.Vec{Y: 8}, worldZ, worldX)
}

// coneAboutY revolves the triangle (far, 0), (far, radius), (apex, 0), drawn
// on the world YZ plane, about U = world Y. Its limit has the readings of the
// same triangle revolved from the world XY plane about X, and A = (0, 1, 0).
func coneAboutY(t *testing.T, doc *Document, far, radius, apex float64) *Body {
	t.Helper()
	return axisConeTool(t, doc, worldY, worldZ, planePoint{U: 1},
		planePoint{U: far}, planePoint{U: far, V: radius}, planePoint{U: apex})
}

// permutedConeTool is pointLoftConeTool mapped by (x, y, z) → (z, x, y).
func permutedConeTool(t *testing.T, doc *Document, apex float64) *Body {
	t.Helper()
	return coneAboutY(t, doc, -2, apex+2, apex)
}

// tiltedTooth lofts the profile on the plane 8 mm from the origin along
// (1, 2, 0)/√5, its U radial in the XY plane and V along world Z.
func tiltedTooth(t *testing.T, doc *Document) *Body {
	t.Helper()
	s5 := math.Sqrt(5)
	return axisTooth(t, doc, r3.Vec{X: 8 / s5, Y: 16 / s5}, r3.Vec{X: -2 / s5, Y: 1 / s5}, worldZ)
}

// tiltedConeTool revolves, on the world XY sketch about the line from (0, 0)
// to (1, 2), the triangle with the far point (-1, -2) and the apex t·(1, 2),
// both exactly on the axis, and the rim (-1, -2) + s·(-2, 1). Its A-scaled
// apex is 5t, its far plane -5 and its rim radius 5s, so the slope is
// (t + 1)/s.
func tiltedConeTool(t *testing.T, doc *Document, apexT, rimS float64) *Body {
	t.Helper()
	return axisConeTool(t, doc, worldX, worldY, planePoint{U: 1, V: 2},
		planePoint{U: -1, V: -2}, planePoint{U: -1 - 2*rimS, V: -2 + rimS}, planePoint{U: apexT, V: 2 * apexT})
}

// The tilted toe has A-scaled apex 11.25 (5.03 mm) and the heel 27.5
// (12.30 mm), both of slope 1, like the +X toe 5 and heel 12.
func tiltedToe(t *testing.T, doc *Document) *Body  { return tiltedConeTool(t, doc, 2.25, 3.25) }
func tiltedHeel(t *testing.T, doc *Document) *Body { return tiltedConeTool(t, doc, 5.5, 6.5) }

func TestPointConeSourceReadsGeneralAxis(t *testing.T) {
	t.Parallel()
	doc := New()
	for _, tc := range []struct {
		name             string
		tool             *Body
		axis             r3.Vec
		apex, far, slope *big.Rat
	}{
		{"x", pointLoftConeTool(t, doc, 5), worldX, big.NewRat(5, 1), big.NewRat(-2, 1), big.NewRat(1, 1)},
		{"permuted", permutedConeTool(t, doc, 5), worldY, big.NewRat(5, 1), big.NewRat(-2, 1), big.NewRat(1, 1)},
		{"tilted toe", tiltedToe(t, doc), r3.Vec{X: 1, Y: 2}, big.NewRat(45, 4), big.NewRat(-5, 1), big.NewRat(1, 1)},
		{"tilted heel", tiltedHeel(t, doc), r3.Vec{X: 1, Y: 2}, big.NewRat(55, 2), big.NewRat(-5, 1), big.NewRat(1, 1)},
	} {
		limit, ok := revolveplan.PointConeSource(&tc.tool.n)
		require.True(t, ok, tc.name)
		require.Equal(t, tc.axis, limit.Axis.Vec(), tc.name)
		require.Equal(t, tc.name == "x", limit.Axis.Canonical(), tc.name)
		require.Zero(t, tc.apex.Cmp(limit.Apex), "%s apex %s", tc.name, limit.Apex)
		require.Zero(t, tc.far.Cmp(limit.Far), "%s far %s", tc.name, limit.Far)
		require.Zero(t, tc.slope.Cmp(limit.Slope), "%s slope %s", tc.name, limit.Slope)
	}
}

func TestPointConeTrimAboutPermutedAxisMatchesX(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		apex    float64
		cut     bool
		ordered bool
	}{
		{name: "toe cut", apex: 5, cut: true},
		{name: "heel inside", apex: 12},
		{name: "toe then heel inside", apex: 12, ordered: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			build := func(tooth *Body, tool func(float64) *Body) *Body {
				if tc.ordered {
					var err error
					tooth, err = Cut(t.Context(), tooth, tool(5))
					require.NoError(t, err)
				}
				op := Intersect
				if tc.cut {
					op = Cut
				}
				got, err := op(t.Context(), tooth, tool(tc.apex))
				require.NoError(t, err)
				return got
			}
			doc := New()
			x := build(pointLoftTooth(t, doc), func(apex float64) *Body { return pointLoftConeTool(t, doc, apex) })
			y := build(permutedTooth(t, doc), func(apex float64) *Body { return permutedConeTool(t, doc, apex) })
			for _, body := range []*Body{x, y} {
				fp, ok := body.n.Payload.(facetedPayload)
				require.True(t, ok)
				require.NotNil(t, fp.PointCone, "the trim must keep its source record")
				requirePointConeAxisMesh(t, body)
			}
			trim := y.n.Payload.(facetedPayload).PointCone
			for _, limit := range []*pointConeLimit{trim.Lower, trim.Upper} {
				if limit != nil {
					require.Equal(t, worldY, limit.Axis.Vec())
				}
			}
			require.Equal(t, faceClasses(x), faceClasses(y))
			xv, err := x.Volume()
			require.NoError(t, err)
			yv, err := y.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(xv.Value.Base()-yv.Value.Base()), xv.Bound.Base()+yv.Bound.Base(),
				"congruent trims must have overlapping volume intervals")
		})
	}
}

func TestPointConeTrimAboutTiltedAxis(t *testing.T) {
	t.Parallel()
	doc := New()
	toeTool := tiltedToe(t, doc)
	toe, err := Cut(t.Context(), tiltedTooth(t, doc), toeTool)
	require.NoError(t, err)
	band, err := Intersect(t.Context(), toe, tiltedHeel(t, doc))
	require.NoError(t, err)
	axis := r3.Vec{X: 1, Y: 2}
	for _, body := range []*Body{toe, band} {
		fp, ok := body.n.Payload.(facetedPayload)
		require.True(t, ok)
		require.NotNil(t, fp.PointCone, "the trim must keep its source record")
		require.Equal(t, axis, fp.PointCone.Lower.Axis.Vec())
		requirePointConeAxisMesh(t, body)
		volume, err := body.Volume()
		require.NoError(t, err)
		require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	}
	require.Equal(t, axis, band.n.Payload.(facetedPayload).PointCone.Upper.Axis.Vec())

	// The toe cone has apex a = 11.25/√5 mm on the axis and slope 1. At the
	// point at axial distance a - ρ and radius ρ on the radial direction
	// e = (-2, 1, 0)/√5, the tool's outward normal is (Â + e)/√2, and the cut
	// face of the trimmed tooth faces the other way.
	s5 := math.Sqrt(5)
	hat, e := r3.Vec{X: 1 / s5, Y: 2 / s5}, r3.Vec{X: -2 / s5, Y: 1 / s5}
	const rho = 2.0
	at := hat.Scale(11.25/s5 - rho).Add(e.Scale(rho))
	want := hat.Add(e).Scale(-1 / math.Sqrt2)
	var cone *Face
	for _, face := range toe.Faces() {
		if _, ok := face.Surface().(Cone); ok {
			require.Nil(t, cone, "the toe cut has one cone face")
			cone = face
		}
	}
	require.NotNil(t, cone)
	normal, err := cone.NormalAt(at)
	require.NoError(t, err)
	require.InDelta(t, want.X, normal.Value.X, 1e-12)
	require.InDelta(t, want.Y, normal.Value.Y, 1e-12)
	require.InDelta(t, want.Z, normal.Value.Z, 1e-12)
}

// requirePointConeAxisMesh tessellates body with every verification and
// requires a cone face and a free-form flank.
func requirePointConeAxisMesh(t *testing.T, body *Body) {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	classes := faceClasses(body)
	require.Positive(t, classes["cone"])
	require.Positive(t, classes["nurbs"])
}

// faceClasses counts body's faces by surface kind.
func faceClasses(body *Body) map[string]int {
	out := map[string]int{}
	for _, face := range body.Faces() {
		switch face.Surface().(type) {
		case Cone:
			out["cone"]++
		case NURBSSurface:
			out["nurbs"]++
		case Plane:
			out["plane"]++
		default:
			out["other"]++
		}
	}
	return out
}

// A limit joins a trim record only about the axis of the limits it already
// holds (agent-docs/revolve-axis-design.md §3.3, §4.4).
func TestPointConeBooleanDeclinesMixedAxes(t *testing.T) {
	t.Parallel()
	doc := New()
	tooth := pointLoftTooth(t, doc)
	d := documentHandle(tooth.n.Doc)
	run := func(op meshbool.OperationKind, target *topo.Body, tool *Body) (*topo.Body, bool) {
		t.Helper()
		out, ok, err := revolveplan.PointConeBoolean(t.Context(), op, envOf(documentNode(d), d.nextProducerID()),
			target, bodyNode(tool), false)
		require.NoError(t, err)
		return out, ok
	}
	xToe, ok := run(meshbool.OpCut, bodyNode(tooth), pointLoftConeTool(t, doc, 5))
	require.True(t, ok)
	_, ok = run(meshbool.OpIntersect, xToe, pointLoftConeTool(t, doc, 12))
	require.True(t, ok, "an X heel joins an X toe")
	_, ok = run(meshbool.OpIntersect, xToe, permutedConeTool(t, doc, 12))
	require.False(t, ok, "a Y heel must not join an X toe")

	yToe, ok := run(meshbool.OpCut, bodyNode(permutedTooth(t, doc)), permutedConeTool(t, doc, 5))
	require.True(t, ok)
	_, ok = run(meshbool.OpIntersect, yToe, permutedConeTool(t, doc, 12))
	require.True(t, ok, "a Y heel joins a Y toe")
	_, ok = run(meshbool.OpIntersect, yToe, pointLoftConeTool(t, doc, 12))
	require.False(t, ok, "an X heel must not join a Y toe")
	// The same cone revolved about the line to (2, 0) records A = (0, 2, 0):
	// parallel to the toe's axis but of another length.
	doubled := axisConeTool(t, doc, worldY, worldZ, planePoint{U: 2},
		planePoint{U: -2}, planePoint{U: -2, V: 14}, planePoint{U: 12})
	limit, ok := revolveplan.PointConeSource(&doubled.n)
	require.True(t, ok)
	require.Equal(t, r3.Vec{Y: 2}, limit.Axis.Vec())
	_, ok = run(meshbool.OpIntersect, yToe, doubled)
	require.False(t, ok, "a heel about a parallel axis of another length must not join")
}

// PointConeSource reads a general cone only from exact on-axis points and a
// coordinate frame.
func TestPointConeSourceRefusesInexactAxisAndFrame(t *testing.T) {
	t.Parallel()
	doc := New()
	exact := tiltedToe(t, doc)
	_, ok := revolveplan.PointConeSource(&exact.n)
	require.True(t, ok, "the exact tilted toe is the control")

	// The apex one unit in the last place off the axis line.
	off := axisConeTool(t, doc, worldX, worldY, planePoint{U: 1, V: 2},
		planePoint{U: -1, V: -2}, planePoint{U: -7.5, V: 1.25}, planePoint{U: 2.25, V: math.Nextafter(4.5, 5)})
	_, ok = revolveplan.PointConeSource(&off.n)
	require.False(t, ok, "an apex off the axis by one ulp must refuse")

	// The +X tool drawn on the XY plane turned 30 degrees about Z: its frame
	// axes are not signed world basis vectors.
	c, s := math.Sqrt(3)/2, 0.5
	turned := axisConeTool(t, doc, r3.Vec{X: c, Y: s}, r3.Vec{X: -s, Y: c}, planePoint{U: 1},
		planePoint{U: -2}, planePoint{U: -2, V: 7}, planePoint{U: 5})
	rp := turned.n.Payload.(revolveplan.Payload)
	require.False(t, coneaxis.FrameCanonical(rp.Frame, rp.Ax))
	_, ok = revolveplan.PointConeSource(&turned.n)
	require.False(t, ok, "a cone on a non-coordinate frame must refuse")
}

// ConeFromFace registers its cone only when PointConeSource reads it back
// about world X (agent-docs/revolve-axis-design.md §4.6).
func TestCommitExactPointConeRefusesGeneralAxis(t *testing.T) {
	t.Parallel()
	doc := New()
	build := func(u, v r3.Vec) *Body {
		t.Helper()
		world := sketch.NewWorld()
		s, err := world.CreateSketch(axisPlane(t, world, r3.Vec{}, u, v))
		require.NoError(t, err)
		corners := []planePoint{{U: -2}, {U: -2, V: 7}, {U: 5}}
		points := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			points[i] = s.CreatePoint(c.U, c.V)
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		built, err := revolveProfileBuild(t.Context(), doc, s, s.Profiles()[0],
			SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{}, nil)
		require.NoError(t, err)
		return built
	}
	before, producer := doc.Bodies(), doc.nextProducerID()
	err := commitExactPointCone(t.Context(), doc, build(worldY, worldZ))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	require.Equal(t, producer, doc.nextProducerID())
	require.NoError(t, commitExactPointCone(t.Context(), doc, build(worldX, worldY)), "the +X cone is the control")
}

// LoftFacesToConeTangent builds its end from +X cone arithmetic, so a cone
// about another axis refuses (agent-docs/revolve-axis-design.md §4.6).
func TestLoftFacesToConeTangentRefusesGeneralAxis(t *testing.T) {
	t.Parallel()
	f := newTangentLoftFixture(t)
	cone := coneAboutY(t, f.doc, -4, 16, 8)
	_, ok := revolveplan.PointConeSource(&cone.n)
	require.True(t, ok, "the cone about Y is an exact point cone")
	_, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)},
		f.toe, f.heel)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "cone about X")
}
