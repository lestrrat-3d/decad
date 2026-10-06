package apitest_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/multibody-dynamics-design.md §13 PR 11. Every
// coordinate is dyadic or the exact image of dyadics under a pose whose float
// entries are read as exact dyadics, so every expected point below is an
// exact rational and each published witness must lie within its own ball of
// it.
//
// Legs shown to fail (each deleted or inverted in turn, fixture red, then
// restored):
//   - the frame's largest-component dropped axis, fixed to z instead:
//     TestContactPairPlanarManifoldWalls reads AmbiguousFeature (both wall
//     faces project to segments, so the clip has zero area);
//   - the counterclockwise orientation of both loops before clipping: a
//     clockwise clip loop keeps nothing, so TestContactPairPlanarManifoldPenetration
//     loses its patch, and TestContactPairPlanarManifoldPlateHole's covered
//     hole falls to the zero-area reading, which publishes corners (the wall
//     pair's clockwise order reads its four corners on that path too);
//   - the hole clip: TestContactPairPlanarManifoldPlateHole's covered hole
//     publishes the outer loop's corners;
//   - the witness conversion ball: TestContactPairPlanarManifoldThirdCrossing's
//     1/3 mm vertex publishes below its own ball;
//   - the normal ball: TestContactPairPlanarManifoldSlopedNormal's (1,0,1)
//     face publishes a unit vector that misses the true normal;
//   - the interior requirement on a supporting face, for a vertex and for an
//     edge: TestContactPairPlanarManifoldRotatedBox's corner on the
//     hexagon's rim, and its edge along the rim, publish points;
//   - the material-side orientation of a crossing: one order of
//     TestContactPairPlanarManifoldEdgeCrossing publishes the normal
//     pointing from B to A;
//   - the penetration patch's unique minimum and its crossing faces:
//     TestContactPairPlanarManifoldPenetration's tied corner and nested box
//     publish patches.
//
// The penetration depth reading reuses the planar gap's square-root
// enclosure, whose half-width leg internal/pair/planar_test.go shows to fail;
// the fixtures here read exact dyadic depths.

// prismBody extrudes a closed polygon h mm along +z.
func prismBody(t *testing.T, doc *decad.Document, pts [][2]float64, h float64) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, pts)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// basisPose is the transform with the given basis and translation.
func basisPose(t *testing.T, ex, ey, ez, at r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.FromBasis(r3.Basis{EX: ex, EY: ey, EZ: ez}, at)
	require.NoError(t, err)
	return pose
}

// rotationPose turns about axis through the origin, then moves the origin to
// at, so a body corner at the origin lands exactly at at.
func rotationPose(t *testing.T, axis r3.Vec, degrees float64, at r3.Vec) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(axis, units.Degrees(degrees))
	require.NoError(t, err)
	pose, err := r3.FromBasis(turn.Basis(), at)
	require.NoError(t, err)
	return pose
}

type ratPoint [3]*big.Rat

func ratAt(x, y, z float64) ratPoint {
	return ratPoint{new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y), new(big.Rat).SetFloat64(z)}
}

func ratFrac(num, den int64) *big.Rat { return big.NewRat(num, den) }

// lineCross is the exact crossing of lines pq and uw in the plane z.
func lineCross(p, q, u, w [2]*big.Rat, z *big.Rat) ratPoint {
	sub := func(a, b [2]*big.Rat) [2]*big.Rat {
		return [2]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1])}
	}
	cross := func(a, b [2]*big.Rat) *big.Rat {
		out := new(big.Rat).Mul(a[0], b[1])
		return out.Sub(out, new(big.Rat).Mul(a[1], b[0]))
	}
	d, e := sub(q, p), sub(w, u)
	t := new(big.Rat).Quo(cross(sub(u, p), e), cross(d, e))
	return ratPoint{new(big.Rat).Add(p[0], new(big.Rat).Mul(t, d[0])),
		new(big.Rat).Add(p[1], new(big.Rat).Mul(t, d[1])), z}
}

func xy(x, y float64) [2]*big.Rat {
	return [2]*big.Rat{new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y)}
}

// within reports whether the exact point lies in the published ball.
func within(want ratPoint, got decad.VecMeasurement) bool {
	value := [3]float64{got.Value.X, got.Value.Y, got.Value.Z}
	squared := new(big.Rat)
	for axis := range 3 {
		d := new(big.Rat).Sub(new(big.Rat).SetFloat64(value[axis]), want[axis])
		squared.Add(squared, d.Mul(d, d))
	}
	bound := new(big.Rat).SetFloat64(got.Bound.Base())
	return squared.Cmp(new(big.Rat).Mul(bound, bound)) <= 0
}

// requireManifoldAt asserts the manifold's points match the exact points in
// any order, one to one, each within its own ball.
func requireManifoldAt(t *testing.T, report *decad.ContactReport, want []ratPoint) {
	t.Helper()
	require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
	require.Equal(t, decad.ContactNoReason, report.Reason)
	require.Len(t, report.Manifold.Points, len(want))
	used := make([]bool, len(want))
	for _, point := range report.Manifold.Points {
		require.Equal(t, point.OnA, point.OnB, "a touch has one witness")
		require.Zero(t, point.Separation.Value.Base())
		require.Zero(t, point.Separation.Bound.Base())
		match := -1
		for i, w := range want {
			if !used[i] && within(w, point.OnA) {
				match = i
				break
			}
		}
		require.GreaterOrEqual(t, match, 0, "published point %v matches no expected point", point.OnA.Value)
		used[match] = true
	}
}

// contactBothWays runs ContactPair in both orders, requires the reversed
// report to swap every witness, face and feature and negate every normal, and
// returns the forward report.
func contactBothWays(t *testing.T, doc *decad.Document, a, b *decad.Body,
	poseA, poseB r3.Transform, req decad.ContactRequest) *decad.ContactReport {
	t.Helper()
	forward, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, req)
	require.NoError(t, err)
	reversed, err := doc.ContactPair(t.Context(), b, a, poseB, poseA, req)
	require.NoError(t, err)
	require.Equal(t, forward.Relation, reversed.Relation)
	require.Equal(t, forward.Reason, reversed.Reason)
	if forward.Manifold == nil {
		require.Nil(t, reversed.Manifold)
		return forward
	}
	require.NotNil(t, reversed.Manifold)
	require.Len(t, reversed.Manifold.Points, len(forward.Manifold.Points))
	for _, point := range forward.Manifold.Points {
		found := false
		for _, other := range reversed.Manifold.Points {
			if other.OnA != point.OnB || other.OnB != point.OnA || other.FaceA != point.FaceB ||
				other.FaceB != point.FaceA || other.FeatureA != point.FeatureB || other.FeatureB != point.FeatureA {
				continue
			}
			require.Equal(t, point.Normal.Value.Scale(-1), other.Normal.Value)
			require.Equal(t, point.Normal.Bound, other.Normal.Bound)
			require.Equal(t, point.NormalAngle, other.NormalAngle)
			require.Equal(t, point.Separation, other.Separation)
			found = true
		}
		require.True(t, found, "reversed manifold lacks %v", point.OnA.Value)
	}
	return forward
}

func TestContactPairPlanarManifoldRotatedBox(t *testing.T) {
	doc := decad.New()
	hex := hexPrismBody(t, doc)
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	id := r3.Identity()
	req := contactRequest()
	top := r3.Vec{X: 10, Y: -2, Z: 12}

	// Turned 50° about (1, -1, 0), all three edges from the box's corner at
	// its origin rise, so that corner alone meets the hexagon's top face.
	vertex := contactBothWays(t, doc, hex, box, id, rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, top), req)
	require.Equal(t, decad.ContactTouching, vertex.Relation)
	requireManifoldAt(t, vertex, []ratPoint{ratAt(10, -2, 12)})
	point := vertex.Manifold.Points[0]
	require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	require.Zero(t, point.NormalAngle.Base())
	require.Equal(t, point.FaceA, point.FeatureA.Face)
	require.Contains(t, hex.Faces(), point.FaceA)
	require.Nil(t, point.FaceB, "a vertex has no single owning face")
	require.NotNil(t, point.FeatureB.Vertex)
	require.Contains(t, box.Vertices(), point.FeatureB.Vertex)

	// Turned 30° about x, the box's edge along x stays level and the rest
	// rises: an edge touch publishes the edge's two ends.
	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	edgePose := basisPose(t, r3.Vec{X: 1}, r3.Vec{Y: c, Z: s}, r3.Vec{Y: -s, Z: c}, top)
	edge := contactBothWays(t, doc, hex, box, id, edgePose, req)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	requireManifoldAt(t, edge, []ratPoint{ratAt(10, -2, 12), ratAt(14, -2, 12)})
	for _, point := range edge.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Nil(t, point.FaceB)
		require.NotNil(t, point.FeatureB.Edge)
		require.Contains(t, box.Edges(), point.FeatureB.Edge)
	}

	// The same corner on the hexagon's rim is a vertex-on-edge contact,
	// whose admissible normals form a cone: no manifold.
	rim := contactBothWays(t, doc, hex, box, id,
		rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, r3.Vec{X: 10, Y: -10, Z: 12}), req)
	require.Equal(t, decad.ContactTouching, rim.Relation)
	require.Nil(t, rim.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, rim.Reason)

	// The level edge laid along the hexagon's rim y = -10 is a parallel
	// edge-on-edge contact: it never reaches the face's interior, so it has
	// no manifold either.
	along := contactBothWays(t, doc, hex, box, id,
		basisPose(t, r3.Vec{X: 1}, r3.Vec{Y: c, Z: s}, r3.Vec{Y: -s, Z: c}, r3.Vec{X: 8, Y: -10, Z: 12}), req)
	require.Equal(t, decad.ContactTouching, along.Relation)
	require.Nil(t, along.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, along.Reason)
}

// rotatedFloorBody is a square floor whose top face is turned in its plane:
// its section corners are exact dyadics, (0,-17) + a·(17.25,10) + b·(-10,17.25)
// for a, b in {0, 1}, and its top face is at z = 10.
func rotatedFloorBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return prismBody(t, doc, [][2]float64{{0, -17}, {17.25, -7}, {7.25, 10.25}, {-10, 0.25}}, 10)
}

func TestContactPairPlanarManifoldHexagonOverhang(t *testing.T) {
	doc := decad.New()
	floor := rotatedFloorBody(t, doc)
	hex := hexPrismBody(t, doc)
	report := contactBothWays(t, doc, floor, hex, r3.Identity(), contactPose(t, r3.Vec{Z: 10}), contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation)
	// The floor's corner (17.25,-7) lies under the hexagon, which overhangs
	// it; the hexagon's corners (0,0) and (5.75,-10) lie on the floor; and
	// four hexagon edges cross the floor's edges.
	z := ratFrac(10, 1)
	h := [][2]*big.Rat{xy(0, 0), xy(5.75, -10), xy(17.25, -10), xy(23, 0), xy(17.25, 10), xy(5.75, 10)}
	f := [][2]*big.Rat{xy(0, -17), xy(17.25, -7), xy(7.25, 10.25), xy(-10, 0.25)}
	want := []ratPoint{
		{h[0][0], h[0][1], z},
		{h[1][0], h[1][1], z},
		lineCross(h[1], h[2], f[0], f[1], z),
		{f[1][0], f[1][1], z},
		lineCross(h[4], h[5], f[1], f[2], z),
		lineCross(h[4], h[5], f[2], f[3], z),
		lineCross(h[5], h[0], f[2], f[3], z),
	}
	requireManifoldAt(t, report, want)
	for _, point := range report.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Contains(t, floor.Faces(), point.FaceA)
		require.Contains(t, hex.Faces(), point.FaceB)
	}

	// The hexagon's corner (0,0) moved onto the floor's corner (17.25,-7),
	// the hexagon beyond it: the coplanar faces meet in one point, a zero-area
	// clip, which publishes that point with the faces' normal.
	corner := contactBothWays(t, doc, floor, hex, r3.Identity(), contactPose(t, r3.Vec{X: 17.25, Y: -7, Z: 10}),
		contactRequest())
	require.Equal(t, decad.ContactTouching, corner.Relation)
	requireManifoldAt(t, corner, []ratPoint{ratAt(17.25, -7, 10)})
	require.Equal(t, r3.Vec{Z: 1}, corner.Manifold.Points[0].Normal.Value)
}

func TestContactPairPlanarManifoldWalls(t *testing.T) {
	doc := decad.New()
	// A house-shaped prism whose x = 10 wall faces a box beside it: the
	// patch is vertical, so its frame drops x.
	house := prismBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {5, 15}, {0, 10}}, 12)
	box := boxBodyAtZ(t, doc, 10, 5, 20, 15, 2, 4)
	report := contactBothWays(t, doc, house, box, r3.Identity(), r3.Identity(), contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation)
	requireManifoldAt(t, report, []ratPoint{ratAt(10, 5, 2), ratAt(10, 10, 2), ratAt(10, 10, 6), ratAt(10, 5, 6)})
	for _, point := range report.Manifold.Points {
		require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
		require.Contains(t, house.Faces(), point.FaceA)
		require.Contains(t, box.Faces(), point.FaceB)
	}
}

// lFloorBody is an L-shaped zero-bound Boolean, its top face at z = 0: the
// square [0,40]² less the corner [10,50]², leaving the arms [0,40]×[0,10] and
// [0,10]×[0,40].
func lFloorBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return cutOf(t, doc, [6]float64{0, 0, 40, 40, -10, 10}, [6]float64{10, 10, 50, 50, -20, 40})
}

func TestContactPairPlanarManifoldNotch(t *testing.T) {
	doc := decad.New()
	floor := lFloorBody(t, doc)
	// A plank laid across the notch, its long edges on x + y = 24 ∓ √2: it
	// meets each arm in its own quadrilateral, reaches past both arms' outer
	// edges, and misses the arms' shared corner at x + y = 20.
	plank := boxBody(t, doc, -24, -1, 24, 1, 3)
	r := math.Sqrt2 / 2
	pose := basisPose(t, r3.Vec{X: r, Y: -r}, r3.Vec{X: r, Y: r}, r3.Vec{Z: 1}, r3.Vec{X: 12, Y: 12})
	report := contactBothWays(t, doc, floor, plank, r3.Identity(), pose, contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	// The plank's exact long edges run through its posed corners.
	basis := pose.Basis()
	corner := func(u, v float64) [2]*big.Rat {
		x := new(big.Rat).Add(ratAt(12, 0, 0)[0], new(big.Rat).Add(
			new(big.Rat).Mul(new(big.Rat).SetFloat64(u), new(big.Rat).SetFloat64(basis.EX.X)),
			new(big.Rat).Mul(new(big.Rat).SetFloat64(v), new(big.Rat).SetFloat64(basis.EY.X))))
		y := new(big.Rat).Add(ratAt(12, 0, 0)[0], new(big.Rat).Add(
			new(big.Rat).Mul(new(big.Rat).SetFloat64(u), new(big.Rat).SetFloat64(basis.EX.Y)),
			new(big.Rat).Mul(new(big.Rat).SetFloat64(v), new(big.Rat).SetFloat64(basis.EY.Y))))
		return [2]*big.Rat{x, y}
	}
	near := [2][2]*big.Rat{corner(-24, -1), corner(24, -1)}
	far := [2][2]*big.Rat{corner(-24, 1), corner(24, 1)}
	z := new(big.Rat)
	var want []ratPoint
	for _, line := range [][2][2]*big.Rat{near, far} {
		for _, cut := range [][2][2]*big.Rat{
			{xy(0, 0), xy(0, 1)}, {xy(10, 0), xy(10, 1)}, {xy(0, 0), xy(1, 0)}, {xy(0, 10), xy(1, 10)},
		} {
			want = append(want, lineCross(line[0], line[1], cut[0], cut[1], z))
		}
	}
	requireManifoldAt(t, report, want)
}

// plateBody is a 40 mm square plate, its top face at z = 0, with a through
// hole [15,25]².
func plateBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return cutOf(t, doc, [6]float64{0, 0, 40, 40, -10, 10}, [6]float64{15, 15, 25, 25, -20, 40})
}

func TestContactPairPlanarManifoldPlateHole(t *testing.T) {
	doc := decad.New()
	plate := plateBody(t, doc)
	over := boxBody(t, doc, 10, 10, 30, 30, 5)
	covered := contactBothWays(t, doc, plate, over, r3.Identity(), r3.Identity(), contactRequest())
	require.Equal(t, decad.ContactTouching, covered.Relation, "reason=%v", covered.Reason)
	require.Nil(t, covered.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, covered.Reason)

	beside := boxBody(t, doc, 26, 5, 36, 15, 5)
	report := contactBothWays(t, doc, plate, beside, r3.Identity(), r3.Identity(), contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation)
	requireManifoldAt(t, report, []ratPoint{ratAt(26, 5, 0), ratAt(36, 5, 0), ratAt(36, 15, 0), ratAt(26, 15, 0)})
}

func TestContactPairPlanarManifoldThirdCrossing(t *testing.T) {
	doc := decad.New()
	// A triangle floor whose edge y = 3x crosses the box's edge y = 1 at
	// x = 1/3, a coordinate no float holds.
	floor := prismBody(t, doc, [][2]float64{{0, 0}, {8, 0}, {1, 3}}, 10)
	box := boxBodyAtZ(t, doc, 0, 0, 4, 1, 10, 2)
	coarse := contactBothWays(t, doc, floor, box, r3.Identity(), r3.Identity(), contactRequest())
	require.Equal(t, decad.ContactTouching, coarse.Relation)
	third := ratPoint{ratFrac(1, 3), ratFrac(1, 1), ratFrac(10, 1)}
	requireManifoldAt(t, coarse, []ratPoint{ratAt(0, 0, 10), ratAt(4, 0, 10), ratAt(4, 1, 10), third})
	var ball float64
	for _, point := range coarse.Manifold.Points {
		ball = math.Max(ball, point.OnA.Bound.Base())
	}
	require.Positive(t, ball, "1/3 mm rounds")

	at := contactRequest()
	at.PointResolution = units.Millimeters(ball)
	exact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), at)
	require.NoError(t, err)
	require.NotNil(t, exact.Manifold, "a ball equal to the request publishes")

	below := contactRequest()
	below.PointResolution = units.Millimeters(ball / 2)
	coarser := contactBothWays(t, doc, floor, box, r3.Identity(), r3.Identity(), below)
	require.Equal(t, decad.ContactTouching, coarser.Relation)
	require.Nil(t, coarser.Manifold)
	require.Equal(t, decad.ContactPointTooCoarse, coarser.Reason)
}

// wedgeBody is a ridge along y: the triangle (-5,0), (5,0), (0,5) extruded
// 20 mm and laid on its side, so its ridge runs over x = 0 at z = 5 for
// y in [-20, 0], between the faces x + z = 5 and z - x = 5.
func wedgeBody(t *testing.T, doc *decad.Document) (*decad.Body, r3.Transform) {
	t.Helper()
	wedge := prismBody(t, doc, [][2]float64{{-5, 0}, {5, 0}, {0, 5}}, 20)
	return wedge, basisPose(t, r3.Vec{X: 1}, r3.Vec{Z: 1}, r3.Vec{Y: -1}, r3.Vec{})
}

func TestContactPairPlanarManifoldEdgeCrossing(t *testing.T) {
	doc := decad.New()
	wedge, wedgePose := wedgeBody(t, doc)
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	// Turned 45° about x, the box's lowest edge runs along x and crosses the
	// ridge at (0, -10, 5).
	r := math.Sqrt2 / 2
	pose := basisPose(t, r3.Vec{X: 1}, r3.Vec{Y: r, Z: r}, r3.Vec{Y: -r, Z: r}, r3.Vec{X: -2, Y: -10, Z: 5})
	report := contactBothWays(t, doc, wedge, box, wedgePose, pose, contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	requireManifoldAt(t, report, []ratPoint{ratAt(0, -10, 5)})
	point := report.Manifold.Points[0]
	require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "the normal leaves the wedge")
	require.NotNil(t, point.FeatureA.Edge)
	require.NotNil(t, point.FeatureB.Edge)
	require.Contains(t, wedge.Edges(), point.FeatureA.Edge)
	require.Contains(t, box.Edges(), point.FeatureB.Edge)

	// The same crossing with the box turned end for end: its lowest edge now
	// runs along -x, which reverses the edges' cross product, and the normal
	// must still leave the wedge.
	turned := basisPose(t, r3.Vec{X: -1}, r3.Vec{Y: -r, Z: r}, r3.Vec{Y: r, Z: r}, r3.Vec{X: 2, Y: -10, Z: 5})
	mirrored := contactBothWays(t, doc, wedge, box, wedgePose, turned, contactRequest())
	require.Equal(t, decad.ContactTouching, mirrored.Relation, "reason=%v", mirrored.Reason)
	requireManifoldAt(t, mirrored, []ratPoint{ratAt(0, -10, 5)})
	require.Equal(t, r3.Vec{Z: 1}, mirrored.Manifold.Points[0].Normal.Value)
}

func TestContactPairPlanarManifoldSlopedNormal(t *testing.T) {
	doc := decad.New()
	wedge, wedgePose := wedgeBody(t, doc)
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	// The box's corner on the sloped face x + z = 5, every edge from it
	// leaving the face.
	pose := rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, r3.Vec{X: 2.5, Y: -10, Z: 2.5})
	report := contactBothWays(t, doc, wedge, box, wedgePose, pose, contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	requireManifoldAt(t, report, []ratPoint{ratAt(2.5, -10, 2.5)})
	point := report.Manifold.Points[0]
	// The true normal is (1, 0, 1)/√2: each component's square is 1/2, which
	// no float reaches, so the ball must be positive and must hold it.
	bound := point.Normal.Bound.Base()
	require.Positive(t, bound)
	require.Positive(t, point.NormalAngle.Base())
	half := ratFrac(1, 2)
	for _, component := range []float64{point.Normal.Value.X, point.Normal.Value.Z} {
		low := new(big.Rat).SetFloat64(component - bound)
		high := new(big.Rat).SetFloat64(component + bound)
		require.LessOrEqual(t, new(big.Rat).Mul(low, low).Cmp(half), 0)
		require.GreaterOrEqual(t, new(big.Rat).Mul(high, high).Cmp(half), 0)
	}
	require.Zero(t, point.Normal.Value.Y)

	tight := contactRequest()
	tight.NormalResolution = units.Radians(point.NormalAngle.Base() / 2)
	withheld := contactBothWays(t, doc, wedge, box, wedgePose, pose, tight)
	require.Equal(t, decad.ContactTouching, withheld.Relation)
	require.Nil(t, withheld.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, withheld.Reason)
}

func TestContactPairPlanarManifoldPenetration(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -40, -40, 40, 40, -10, 10)
	hex := hexPrismBody(t, doc)
	// Sunk 0.5 mm: the floor's top and the hexagon's bottom are the unique
	// shallowest crossing, and the patch is the whole hexagon at depth.
	report := contactBothWays(t, doc, floor, hex, r3.Identity(), contactPose(t, r3.Vec{Z: -0.5}), contactRequest())
	require.Equal(t, decad.ContactOverlapping, report.Relation)
	require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
	require.Len(t, report.Manifold.Points, 6)
	corners := []r3.Vec{{}, {X: 5.75, Y: -10}, {X: 17.25, Y: -10}, {X: 23}, {X: 17.25, Y: 10}, {X: 5.75, Y: 10}}
	for _, point := range report.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Zero(t, point.OnA.Value.Z)
		require.Equal(t, -0.5, point.OnB.Value.Z)
		require.Equal(t, point.OnA.Value.X, point.OnB.Value.X)
		require.Equal(t, point.OnA.Value.Y, point.OnB.Value.Y)
		require.Contains(t, corners, r3.Vec{X: point.OnA.Value.X, Y: point.OnA.Value.Y})
		require.Equal(t, -0.5, point.Separation.Value.Base())
		require.Zero(t, point.Separation.Bound.Base())
	}

	// Sunk as deep into the floor's side as into its top: the floor ends at
	// x = 0.5, past the hexagon's corner at x = 0, so moving the hexagon up
	// or along +x clears it by the same 0.5 mm, and no patch is published.
	edge := boxBodyAtZ(t, doc, -40, -40, 0.5, 40, -10, 10)
	tied := contactBothWays(t, doc, edge, hex, r3.Identity(), contactPose(t, r3.Vec{Z: -0.5}), contactRequest())
	require.Equal(t, decad.ContactOverlapping, tied.Relation)
	require.Nil(t, tied.Manifold)

	// The hexagon wholly inside a slab has a unique shallowest translation,
	// down through the slab's bottom, but the hexagon's top face does not
	// cross the slab's: no patch.
	slab := boxBodyAtZ(t, doc, -10, -20, 40, 20, -5, 25)
	nested := contactBothWays(t, doc, slab, hex, r3.Identity(), r3.Identity(), contactRequest())
	require.Equal(t, decad.ContactOverlapping, nested.Relation)
	require.Nil(t, nested.Manifold)
}

// TestPlanarManifoldStitchedVertexTouch publishes the stitched tetrahedron's
// manifolds on a source-box floor whose top face is z = 0
// (docs/multibody-dynamics-design.md §13 PR 14b). Deleting the stitched
// solid's face map (Faces nil) was shown to fail: the apex keeps
// ContactTouching with no manifold.
func TestPlanarManifoldStitchedVertexTouch(t *testing.T) {
	doc := decad.New()
	tetrahedron, a := stitchedTetrahedron(t, doc)
	floor := boxBodyAtZ(t, doc, -40, -40, 40, 40, -10, 10)
	id := r3.Identity()

	// Turned 50° about (1, -1, 0), every edge from the corner at the origin
	// rises (each other corner's height is a positive mix of cos 50° and
	// sin 50°), so that corner stands alone on the floor at its staged
	// position, the pose's translation.
	at := r3.Vec{X: 3, Y: -2}
	apex := contactBothWays(t, doc, floor, tetrahedron, id, rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, at),
		contactRequest())
	require.Equal(t, decad.ContactTouching, apex.Relation, "reason=%v", apex.Reason)
	requireManifoldAt(t, apex, []ratPoint{ratAt(3, -2, 0)})
	point := apex.Manifold.Points[0]
	require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	require.Zero(t, point.NormalAngle.Base())
	require.Contains(t, floor.Faces(), point.FaceA)
	require.Nil(t, point.FaceB, "a vertex has no single owning face")
	require.NotNil(t, point.FeatureB.Vertex)
	require.Contains(t, tetrahedron.Vertices(), point.FeatureB.Vertex)
	require.Equal(t, r3.Vec{}, point.FeatureB.Vertex.Position().Value, "the live apex")

	// On its XY face: the clip is the whole face, its three corners.
	face := contactBothWays(t, doc, floor, tetrahedron, id, id, contactRequest())
	require.Equal(t, decad.ContactTouching, face.Relation, "reason=%v", face.Reason)
	requireManifoldAt(t, face, []ratPoint{ratAt(a, 0, 0), ratAt(0, 0, 0), ratAt(0, a, 0)})
	for _, point := range face.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Contains(t, floor.Faces(), point.FaceA)
		require.Contains(t, tetrahedron.Faces(), point.FaceB)
	}
}

func TestContactPairPlanarManifoldCancels(t *testing.T) {
	doc := decad.New()
	floor := rotatedFloorBody(t, doc)
	hex := hexPrismBody(t, doc)
	pose := contactPose(t, r3.Vec{Z: 10})
	// The first query caches both convexity certificates; the second counts
	// the polls of a query that clips and publishes the seven corners.
	var polls int32
	for range 2 {
		counting := newCancelAfterContext(t.Context(), math.MaxInt32)
		report, err := doc.ContactPair(counting, floor, hex, r3.Identity(), pose, contactRequest())
		require.NoError(t, err)
		require.NotNil(t, report.Manifold)
		polls = counting.calls.Load()
	}
	before := doc.Bodies()
	// Every poll, the phase poll after the manifold included, stops the
	// query with the context's error and no report. The kernel's own polls
	// inside the clip are cancelled call by call in internal/pair.
	for limit := int32(1); limit <= polls; limit++ {
		canceling := newCancelAfterContext(t.Context(), limit)
		report, err := doc.ContactPair(canceling, floor, hex, r3.Identity(), pose, contactRequest())
		require.ErrorIs(t, err, context.Canceled, "limit=%d", limit)
		require.Nil(t, report)
	}
	require.Equal(t, before, doc.Bodies())
}

// The fixtures of docs/multibody-dynamics-design.md §13 PR 14d: §9.6's
// face-local patch of the 8 mm cube poking through the §2 tray's floor, whose
// top face is z = 0. The tray is not convex, so §9.3's patch never runs; the
// cube is. Each runs in both body orders.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - condition 1, one crossed face:
//     TestPlanarManifoldWithholdsCornerThroughTwoFaces reads
//     ContactNoNormalProof instead of ContactAmbiguousFeature (the corner's
//     foot lies past the floor and the cube reaches over the wall's foot, so
//     conditions 3 and 4 still refuse it); internal/pair's cavity fixture
//     shows the leg publishing a corner;
//   - the lifted set under a positive SupportBand:
//     TestPlanarManifoldCornerThroughTrayFloor's barely turned cube publishes
//     its sunk corner alone.
//
// Conditions 2 to 4 are shown to fail at the snapshot level
// (internal/pair/planar_face_penetration_test.go and its internal test).

// trayCubeTwoWays runs ContactPair over the §2 tray at the identity and the
// cube at pose in both body orders and returns the report with the tray as A.
func trayCubeTwoWays(t *testing.T, doc *decad.Document, tray, cube *decad.Body, pose r3.Transform,
	req decad.ContactRequest) *decad.ContactReport {
	t.Helper()
	return contactBothWays(t, doc, tray, cube, r3.Identity(), pose, req)
}

// requireSunkPoint checks one published point of the cube poking through
// the floor: the cube's witness is the exact point at, the tray's its foot on
// z = 0, the normal is the floor's +z and Separation is exactly at's height.
func requireSunkPoint(t *testing.T, point decad.ContactPoint, at r3.Vec) {
	t.Helper()
	require.Equal(t, at, point.OnB.Value)
	require.Zero(t, point.OnB.Bound.Base())
	require.Equal(t, r3.Vec{X: at.X, Y: at.Y}, point.OnA.Value)
	require.Zero(t, point.OnA.Bound.Base())
	require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	require.Zero(t, point.NormalAngle.Base())
	require.Equal(t, at.Z, point.Separation.Value.Base())
	require.Zero(t, point.Separation.Bound.Base())
	require.NotNil(t, point.FaceA)
	require.Equal(t, point.FaceA, point.FeatureA.Face)
}

func TestPlanarManifoldCornerThroughTrayFloor(t *testing.T) {
	const depth = 1.0 / (1 << 20)
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cube := boxBody(t, doc, 0, 0, 8, 8, 8)
	at := r3.Vec{X: 1, Y: 2, Z: -depth}

	// Turned 30° about (1, -1, 0), every edge from the corner at the cube's
	// origin rises, so that corner, staged exactly at at, is the one deepest
	// vertex, 2⁻²⁰ mm below the floor.
	pose := rotationPose(t, r3.Vec{X: 1, Y: -1}, 30, at)
	for _, req := range []decad.ContactRequest{contactRequest(), supportBandRequest(2 * depth)} {
		report := trayCubeTwoWays(t, doc, tray, cube, pose, req)
		require.Equal(t, decad.ContactOverlapping, report.Relation)
		require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
		require.Equal(t, decad.ContactNoReason, report.Reason)
		// Every other corner stands millimetres up, outside any band.
		require.Len(t, report.Manifold.Points, 1)
		point := report.Manifold.Points[0]
		requireSunkPoint(t, point, at)
		require.Contains(t, tray.Faces(), point.FaceA)
		require.Nil(t, point.FaceB, "a vertex has no single owning face")
		require.NotNil(t, point.FeatureB.Vertex)
		require.Equal(t, r3.Vec{}, point.FeatureB.Vertex.Position().Value, "the live corner")
	}

	// Barely turned, the same corner is still the deepest, and the cube's
	// other three bottom corners stand within 2⁻¹⁹ mm of the floor. A
	// positive band publishes them after the sunk corner, each at its exact
	// height, read off the posed basis.
	slight := rotationPose(t, r3.Vec{X: 1, Y: -1}, 5e-6, at)
	alone := trayCubeTwoWays(t, doc, tray, cube, slight, contactRequest())
	require.NotNil(t, alone.Manifold, "reason=%v", alone.Reason)
	require.Len(t, alone.Manifold.Points, 1)
	requireSunkPoint(t, alone.Manifold.Points[0], at)

	banded := trayCubeTwoWays(t, doc, tray, cube, slight, supportBandRequest(2*depth))
	require.NotNil(t, banded.Manifold, "reason=%v", banded.Reason)
	require.Len(t, banded.Manifold.Points, 4)
	requireSunkPoint(t, banded.Manifold.Points[0], at)
	basis := slight.Basis()
	var want []*big.Rat
	for _, corner := range [][2]float64{{8, 0}, {0, 8}, {8, 8}} {
		height := new(big.Rat).SetFloat64(at.Z)
		height.Add(height, new(big.Rat).Mul(new(big.Rat).SetFloat64(corner[0]), new(big.Rat).SetFloat64(basis.EX.Z)))
		height.Add(height, new(big.Rat).Mul(new(big.Rat).SetFloat64(corner[1]), new(big.Rat).SetFloat64(basis.EY.Z)))
		require.Positive(t, height.Cmp(new(big.Rat).SetFloat64(at.Z)), "above the sunk corner")
		require.Negative(t, height.Cmp(new(big.Rat).SetFloat64(2*depth)), "inside the band")
		want = append(want, height)
	}
	used := make([]bool, len(want))
	for _, point := range banded.Manifold.Points[1:] {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.NotNil(t, point.FeatureB.Vertex)
		value := new(big.Rat).SetFloat64(point.Separation.Value.Base())
		bound := new(big.Rat).SetFloat64(point.Separation.Bound.Base())
		match := -1
		for i, w := range want {
			if !used[i] && new(big.Rat).Abs(new(big.Rat).Sub(value, w)).Cmp(bound) <= 0 {
				match = i
				break
			}
		}
		require.GreaterOrEqual(t, match, 0, "lifted point at %v matches no corner height", point.Separation.Value)
		used[match] = true
	}
}

func TestPlanarManifoldEdgeThroughTrayFloor(t *testing.T) {
	const depth = 1.0 / (1 << 20)
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cube := boxBody(t, doc, 0, -4, 8, 4, 8)
	at := r3.Vec{X: 1, Y: 2, Z: -depth}
	// Turned 30° about -y, the cube's edge x = z = 0 stays level, staged
	// at the pose's origin ± 4 along y, and the rest rises.
	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	pose := basisPose(t, r3.Vec{X: c, Z: s}, r3.Vec{Y: 1}, r3.Vec{X: -s, Z: c}, at)
	report := trayCubeTwoWays(t, doc, tray, cube, pose, contactRequest())
	require.Equal(t, decad.ContactOverlapping, report.Relation)
	require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
	require.Len(t, report.Manifold.Points, 2)
	ends := []r3.Vec{{X: 1, Y: -2, Z: -depth}, {X: 1, Y: 6, Z: -depth}}
	for _, point := range report.Manifold.Points {
		require.Contains(t, ends, point.OnB.Value)
		requireSunkPoint(t, point, point.OnB.Value)
		require.NotNil(t, point.FeatureB.Edge)
		require.Contains(t, cube.Edges(), point.FeatureB.Edge)
	}
	require.NotEqual(t, report.Manifold.Points[0].OnB.Value, report.Manifold.Points[1].OnB.Value)
}

func TestPlanarManifoldWithholdsCornerThroughTwoFaces(t *testing.T) {
	const depth = 1.0 / (1 << 20)
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cube := boxBody(t, doc, 0, 0, 8, 8, 8)
	// The corner below the floor and past the wall x = -80, in the material
	// where the two meet: the cube crosses both faces.
	pose := rotationPose(t, r3.Vec{X: 1, Y: -1}, 30, r3.Vec{X: -80 - depth, Z: -depth})
	report := trayCubeTwoWays(t, doc, tray, cube, pose, contactRequest())
	require.Equal(t, decad.ContactOverlapping, report.Relation)
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, report.Reason)
}

// The fixtures of docs/multibody-dynamics-design.md §13 PR 20d: §10.5's
// support set of a non-convex guest, the §2 cup, over the §2 tray. Neither
// body carries the convexity certificate, so §9.3 never runs; every fixture
// runs in both body orders. Every cup corner below is the exact image of a
// dyadic corner under a pose whose entries are dyadic, so each point and
// height is compared exactly.
//
// Legs shown to fail (each restored or deleted in turn, fixture red, then
// put back):
//   - the convexity gate on a touch (ContactNonConvex whenever neither body
//     is convex), restored: the cup on the floor and the cup on the rim
//     publish no manifold;
//   - the convexity gate on the lifted set (a plane's guest must be convex),
//     restored: the tilted cup reads Separated with no manifold;
//   - the host-face test on every contact, deleted (every flat host face a
//     candidate): the cup against the wall publishes the corners strictly
//     inside the floor and the wall, under two normals;
//   - the foot test (a contact vertex strictly inside the host face),
//     deleted: the cup on the rim loses its manifold to
//     ContactAmbiguousFeature, since the cup's bottom face then also hosts
//     the rim's own corners outside it, and
//     TestPlanarManifoldNonConvexGuestBehind publishes the step's corners;
//   - the guest-in-front test, deleted: TestPlanarManifoldNonConvexGuestBehind
//     publishes the arm's two corners.

// cupCorners are the cup's four bottom corners in its own frame.
var cupCorners = []r3.Vec{{X: -12, Y: -8}, {X: 12, Y: -8}, {X: 12, Y: 8}, {X: -12, Y: 8}}

// requireCupCorner checks that a manifold point's cup feature is the live
// corner vertex at body-frame position corner.
func requireCupCorner(t *testing.T, cup *decad.Body, point decad.ContactPoint, corner r3.Vec) {
	t.Helper()
	require.Nil(t, point.FaceB, "a vertex has no single owning face")
	require.NotNil(t, point.FeatureB.Vertex)
	require.Contains(t, cup.Vertices(), point.FeatureB.Vertex)
	require.Equal(t, corner, point.FeatureB.Vertex.Position().Value, "the live corner")
}

func TestPlanarManifoldNonConvexGuest(t *testing.T) {
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cup := partsBinCup(t, doc)

	// On the floor at a translation pose, the cup's bottom face lies flat on
	// the floor's: every contact is on the floor, and the floor publishes the
	// four bottom corners at their exact coordinates.
	t.Run("floor", func(t *testing.T) {
		at := r3.Vec{X: 3, Y: -2}
		floor := contactBothWays(t, doc, tray, cup, r3.Identity(), contactPose(t, at), contactRequest())
		require.Equal(t, decad.ContactTouching, floor.Relation, "reason=%v", floor.Reason)
		want := make([]ratPoint, 0, len(cupCorners))
		for _, c := range cupCorners {
			want = append(want, ratAt(c.X+at.X, c.Y+at.Y, 0))
		}
		requireManifoldAt(t, floor, want)
		for _, point := range floor.Manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			require.Zero(t, point.NormalAngle.Base())
			require.Zero(t, point.OnA.Bound.Base())
			require.Contains(t, tray.Faces(), point.FaceA)
			requireCupCorner(t, cup, point, r3.Vec{X: point.OnB.Value.X - at.X, Y: point.OnB.Value.Y - at.Y})
		}
	})

	// Turned about Y by sin θ = 2⁻²⁴ and lifted so its low corners stand
	// 2⁻³⁰ mm up, the cup's far corners stand 24·sin θ higher, 1.5·2⁻²⁰ mm,
	// inside a 2⁻¹⁹ mm band. The pair is a ContactBand whose manifold is the
	// floor's lifted set: the four bottom corners at their exact heights.
	t.Run("tilted", func(t *testing.T) {
		const lift = 1.0 / (1 << 30)
		tilted := basisPose(t, r3.Vec{X: supportCosine, Z: supportSine}, r3.Vec{Y: 1},
			r3.Vec{X: -supportSine, Z: supportCosine}, r3.Vec{Z: 12*supportSine + lift})
		band := contactBothWays(t, doc, tray, cup, r3.Identity(), tilted, supportBandRequest(1.0/(1<<19)))
		require.Equal(t, decad.ContactBand, band.Relation, "reason=%v", band.Reason)
		require.Equal(t, decad.ContactNoReason, band.Reason)
		require.Zero(t, band.Gap.Value.Base())
		require.Equal(t, lift, band.Gap.Bound.Base())
		require.NotNil(t, band.Manifold)
		require.Len(t, band.Manifold.Points, 4)
		heights := map[float64]int{}
		for _, point := range band.Manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			require.Zero(t, point.Separation.Bound.Base())
			height := point.Separation.Value.Base()
			heights[height]++
			require.Equal(t, height, point.OnB.Value.Z, "the corner stands at its Separation")
			require.Equal(t, r3.Vec{X: point.OnB.Value.X, Y: point.OnB.Value.Y}, point.OnA.Value, "its foot")
			require.NotNil(t, point.FeatureB.Vertex)
			require.Contains(t, cup.Vertices(), point.FeatureB.Vertex)
		}
		require.Equal(t, map[float64]int{lift: 2, 24*supportSine + lift: 2}, heights)
	})

	// On the tray's rim, x ∈ [80, 90] at z = 40, with two corners past its
	// outer edge x = 90: every contact lies on the rim's face, and only the two
	// corners over it publish.
	t.Run("rim", func(t *testing.T) {
		rimAt := r3.Vec{X: 94, Y: 68, Z: 40}
		rim := contactBothWays(t, doc, tray, cup, r3.Identity(), contactPose(t, rimAt), contactRequest())
		require.Equal(t, decad.ContactTouching, rim.Relation, "reason=%v", rim.Reason)
		requireManifoldAt(t, rim, []ratPoint{ratAt(82, 60, 40), ratAt(82, 76, 40)})
		for _, point := range rim.Manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			require.Contains(t, tray.Faces(), point.FaceA)
			requireCupCorner(t, cup, point, r3.Vec{X: point.OnB.Value.X - rimAt.X, Y: point.OnB.Value.Y - rimAt.Y})
		}
	})

	// Standing on the floor with its side flat against the wall x = -80, the
	// cup touches two faces of the tray: no single face holds every contact.
	t.Run("wall", func(t *testing.T) {
		wall := contactBothWays(t, doc, tray, cup, r3.Identity(), contactPose(t, r3.Vec{X: -68}), contactRequest())
		require.Equal(t, decad.ContactTouching, wall.Relation, "reason=%v", wall.Reason)
		require.Nil(t, wall.Manifold)
		require.Equal(t, decad.ContactNonConvex, wall.Reason)
	})
}

// TestPlanarManifoldNonConvexGuestBehind is an inverted-L guest whose arm
// lies flat on a face of an L-shaped step, its leg hanging past the step's
// end and below that face's plane without meeting the step. Every contact
// lies on the step's face, but the guest is not wholly in front of its
// plane, so the face supports nothing, and no face of the guest holds a step
// corner strictly inside it: no manifold.
func TestPlanarManifoldNonConvexGuestBehind(t *testing.T) {
	doc := decad.New()
	step := prismBody(t, doc, [][2]float64{{0, 0}, {20, 0}, {20, 5}, {10, 5}, {10, 10}, {0, 10}}, 10)
	guest := prismBody(t, doc, [][2]float64{{-5, 0}, {-2, 0}, {-2, 10}, {8, 10}, {8, 12}, {-5, 12}}, 6)
	report := contactBothWays(t, doc, step, guest, r3.Identity(), contactPose(t, r3.Vec{Z: 2}), contactRequest())
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactNonConvex, report.Reason)
}
