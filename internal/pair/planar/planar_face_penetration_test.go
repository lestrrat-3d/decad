package planar_test

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// The face-local penetration patch of docs/multibody-dynamics-design.md §9.6
// at the snapshot level. Every coordinate is a small dyadic, so every test is
// decided exactly.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - condition 1, one crossed face: TestPlanarFacePenetrationCavity's corner
//     sunk through the floor's skin into the cavity below publishes the
//     corner at its full depth, although its deepest point lies in the
//     cavity;
//   - condition 4, the column test: TestPlanarFacePenetrationColumn's tray
//     with a 1 mm cube standing on its floor wholly inside the sunk box
//     publishes one corner, although the box also meets that cube.
//
// Condition 3's region test is shown to fail on its helper
// (planar_face_penetration_internal_test.go): a sunk part that leaves the
// crossed face generically crosses the face's rim, which condition 1 refuses
// first. Condition 2 is shallowSupport's, whose legs planar_patch_test.go
// shows.

// faceWound appends the quad q0..q3 as two triangles wound so their normals
// point along out, all owned by face.
func faceWound(s *planar.PlanarSolid, face int, q [4]int, out proof.DyV3) {
	for _, tri := range [][3]int{{q[0], q[1], q[2]}, {q[0], q[2], q[3]}} {
		a := s.Verts[tri[0]]
		n := proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
		if proof.DvDot(n, out).Sign() < 0 {
			tri[1], tri[2] = tri[2], tri[1]
		}
		s.Tris = append(s.Tris, tri)
		s.Faces = append(s.Faces, face)
	}
}

// square returns the corners of [-r, r]² counterclockwise at height z.
func square(r, z float64) [][3]float64 {
	return [][3]float64{{-r, -r, z}, {r, -r, z}, {r, r, z}, {-r, r, z}}
}

// sideOut is the outward normal of the square's side from corner k to k+1.
var sideOut = []proof.DyV3{vec(0, -1, 0), vec(1, 0, 0), vec(0, 1, 0), vec(-1, 0, 0)}

// faceTray is a tray: the outer box [-16,16]²×[-2,8] less the pocket
// [-12,12]²×[0,8]. Faces: 0 the bottom, 1-4 the outer walls, 5 the rim,
// 6-9 the inner walls, 10 the floor at z = 0, whose diagonal runs from
// (-12,-12) to (12,12).
func faceTray() planar.PlanarSolid {
	var s planar.PlanarSolid
	for _, ring := range [][][3]float64{square(16, -2), square(16, 8), square(12, 8), square(12, 0)} {
		for _, p := range ring {
			s.Verts = append(s.Verts, vec(p[0], p[1], p[2]))
		}
	}
	o, top, in, floor := 0, 4, 8, 12
	faceWound(&s, 0, [4]int{o, o + 1, o + 2, o + 3}, vec(0, 0, -1))
	for k := range 4 {
		j := (k + 1) % 4
		faceWound(&s, 1+k, [4]int{o + k, o + j, top + j, top + k}, sideOut[k])
		faceWound(&s, 5, [4]int{top + k, top + j, in + j, in + k}, vec(0, 0, 1))
		faceWound(&s, 6+k, [4]int{in + k, in + j, floor + j, floor + k}, dvNegate(sideOut[k]))
	}
	faceWound(&s, 10, [4]int{floor, floor + 1, floor + 2, floor + 3}, vec(0, 0, 1))
	return s
}

const trayFloor = 10

func dvNegate(v proof.DyV3) proof.DyV3 {
	return proof.DyV3{proof.DyNeg(v[0]), proof.DyNeg(v[1]), proof.DyNeg(v[2])}
}

// boxFaces names the faces of boxTris' twelve triangles: x low, x high,
// y low, y high, z low, z high.
var boxFaces = []int{0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5}

// parallelepiped is the convex solid c + a·u + b·v + g·w for a, b, g in
// [0, 1], vertex i at bits (a, b, g) = (i&1, i>>1&1, i>>2&1), with boxFaces.
func parallelepiped(c, u, v, w [3]float64) planar.PlanarSolid {
	points := make([][3]float64, 8)
	for i := range points {
		for axis := range 3 {
			points[i][axis] = c[axis] + float64(i&1)*u[axis] + float64(i>>1&1)*v[axis] + float64(i>>2&1)*w[axis]
		}
	}
	s := hullSolid(points, boxTris(0))
	s.Faces = append([]int(nil), boxFaces...)
	return s
}

// tiny is the corner depth below the floor, 2⁻²⁰ mm.
const tiny = 1.0 / (1 << 20)

// cornerDown is a parallelepiped whose corner c is its lowest vertex: every
// edge from it rises.
func cornerDown(c [3]float64) planar.PlanarSolid {
	return parallelepiped(c, [3]float64{4, 0, 1}, [3]float64{0, 4, 1}, [3]float64{-1, -1, 4})
}

func audited(t *testing.T, solids ...*planar.PlanarSolid) {
	t.Helper()
	for _, s := range solids {
		ok, err := planar.CheckPlanarSolid(s, noPoll)
		require.NoError(t, err)
		require.True(t, ok)
	}
}

// facePenetration classifies the pair, requires Overlapping, and returns the
// face-local patch with the given convexity flags.
func facePenetration(t *testing.T, a, b planar.PlanarSolid, convexA, convexB bool) planar.PlanarManifold {
	t.Helper()
	audited(t, &a, &b)
	result, err := planar.ClassifyPlanar(&a, &b, noPoll)
	require.NoError(t, err)
	require.Equal(t, pair.Overlapping, result.Relation)
	got, err := planar.PlanarFacePenetration(&a, &b, result.Crossings, convexA, convexB, noPoll)
	require.NoError(t, err)
	return got
}

func ratPt(x, y, z float64) planar.Point3 {
	return planar.Point3{new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y), new(big.Rat).SetFloat64(z)}
}

func requirePoint3(t *testing.T, want, got planar.Point3) {
	t.Helper()
	for axis := range 3 {
		require.Zero(t, want[axis].Cmp(got[axis]), "axis %d: want %s, got %s", axis,
			want[axis].RatString(), got[axis].RatString())
	}
}

func TestPlanarFacePenetrationCorner(t *testing.T) {
	tray := faceTray()
	c := [3]float64{2, 3, -tiny}
	box := cornerDown(c)
	for _, boxIsA := range []bool{true, false} {
		a, b := box, tray
		if !boxIsA {
			a, b = tray, box
		}
		got := facePenetration(t, a, b, boxIsA, !boxIsA)
		require.Len(t, got.Points, 1, "reason=%v", got.Reason)
		point := got.Points[0]
		onBox, onTray := point.OnA, point.OnB
		if !boxIsA {
			onBox, onTray = point.OnB, point.OnA
		}
		requirePoint3(t, ratPt(c[0], c[1], c[2]), onBox)
		requirePoint3(t, ratPt(c[0], c[1], 0), onTray)
		require.Equal(t, pair.ScalarReading{ValueMM: -tiny}, point.Separation)
		// The floor's outward normal, oriented A to B.
		require.True(t, point.Normal[0].Sign() == 0 && point.Normal[1].Sign() == 0)
		if boxIsA {
			require.Negative(t, point.Normal[2].Sign())
		} else {
			require.Positive(t, point.Normal[2].Sign())
		}
		require.Equal(t, []planar.SupportPlane{{HostIsA: !boxIsA, Face: trayFloor}}, got.Supports)
	}

	// An edge down: the box's edge from c along y stays at c's depth, and
	// both of its ends are published.
	edge := parallelepiped(c, [3]float64{4, 0, 1}, [3]float64{0, 4, 0}, [3]float64{-1, 0, 4})
	got := facePenetration(t, edge, tray, true, false)
	require.Len(t, got.Points, 2, "reason=%v", got.Reason)
	requirePoint3(t, ratPt(c[0], c[1], c[2]), got.Points[0].OnA)
	requirePoint3(t, ratPt(c[0], c[1]+4, c[2]), got.Points[1].OnA)
	requirePoint3(t, ratPt(c[0], c[1]+4, 0), got.Points[1].OnB)
}

func TestPlanarFacePenetrationTwoFaces(t *testing.T) {
	// The corner below the floor and past the inner wall x = 12: the box
	// crosses both faces.
	box := parallelepiped([3]float64{12 + 1.0/1024, 0, -1.0 / 1024},
		[3]float64{-4, 0, 1}, [3]float64{0, 4, 1}, [3]float64{-1, -1, 4})
	got := facePenetration(t, box, faceTray(), true, false)
	require.Nil(t, got.Points)
	require.Equal(t, pair.AmbiguousFeature, got.Reason)
}

// cavitySlab is the slab [-16,16]²×[-4,0] with the closed cavity
// [-8,8]²×[-3,-skin] under its top face (face 5).
func cavitySlab(skin float64) planar.PlanarSolid {
	s := hollowSolid([3]float64{-16, -16, -4}, [3]float64{16, 16, 0}, [3]float64{-8, -8, -3}, [3]float64{8, 8, -skin})
	s.Faces = append(append([]int(nil), boxFaces...), boxFaces...)
	for i := len(boxFaces); i < len(s.Faces); i++ {
		s.Faces[i] += 6
	}
	return s
}

func TestPlanarFacePenetrationCavity(t *testing.T) {
	const skin = 1.0 / 1024
	slab := cavitySlab(skin)
	// Sunk less than the skin, the corner crosses the top face alone.
	shallow := facePenetration(t, cornerDown([3]float64{2, 3, -skin / 2}), slab, true, false)
	require.Len(t, shallow.Points, 1, "reason=%v", shallow.Reason)

	// Sunk through the skin, it also crosses the cavity's ceiling: its
	// deepest point lies in the cavity, not in the slab's material.
	through := facePenetration(t, cornerDown([3]float64{2, 3, -2 * skin}), slab, true, false)
	require.Nil(t, through.Points)
	require.Equal(t, pair.AmbiguousFeature, through.Reason)
}

// withShell appends a second closed shell to s, its faces numbered after
// s's.
func withShell(s, shell planar.PlanarSolid) planar.PlanarSolid {
	base, faceBase := len(s.Verts), 0
	for _, f := range s.Faces {
		faceBase = max(faceBase, f+1)
	}
	out := planar.PlanarSolid{Verts: append(append([]proof.DyV3(nil), s.Verts...), shell.Verts...),
		Tris: append([][3]int(nil), s.Tris...), Faces: append([]int(nil), s.Faces...)}
	for i, tri := range shell.Tris {
		out.Tris = append(out.Tris, [3]int{tri[0] + base, tri[1] + base, tri[2] + base})
		out.Faces = append(out.Faces, faceBase+shell.Faces[i])
	}
	return out
}

func TestPlanarFacePenetrationColumn(t *testing.T) {
	// A box whose bottom face dips below the floor everywhere, lowest at its
	// corner (1, 0.5, -2⁻¹⁸) and rising by 2⁻²⁰ along x and 2⁻²¹ along y.
	box := parallelepiped([3]float64{1, 0.5, -4 * tiny},
		[3]float64{8, 0, tiny}, [3]float64{0, 8, tiny / 2}, [3]float64{0, 0, 8})
	tray := faceTray()
	plain := facePenetration(t, box, tray, true, false)
	require.Len(t, plain.Points, 1, "reason=%v", plain.Reason)
	requirePoint3(t, ratPt(1, 0.5, -4*tiny), plain.Points[0].OnA)

	// A 1 mm cube standing on the floor wholly inside the sunk box: no
	// crossing reaches it, so only the column test sees that the box meets it.
	cube := boxSolid([3]float64{4, 4, 0}, [3]float64{5, 5, 1})
	cube.Faces = append([]int(nil), boxFaces...)
	got := facePenetration(t, box, withShell(tray, cube), true, false)
	require.Nil(t, got.Points)
	require.Equal(t, pair.NoReason, got.Reason)
}

func TestPlanarFacePenetrationNeedsCrossings(t *testing.T) {
	// A box nested in a hollow solid's material: Overlapping by the parity
	// cast alone, with no crossing to name a face.
	hollow := hollowSolid([3]float64{-20, -20, -20}, [3]float64{20, 20, 20},
		[3]float64{-15, -15, -15}, [3]float64{15, 15, 15})
	hollow.Faces = make([]int, len(hollow.Tris))
	box := boxSolid([3]float64{16, -1, -1}, [3]float64{18, 1, 1})
	box.Faces = append([]int(nil), boxFaces...)
	result := classify(t, box, hollow)
	require.Equal(t, pair.Overlapping, result.Relation)
	require.Empty(t, result.Crossings)
	got, err := planar.PlanarFacePenetration(&box, &hollow, result.Crossings, true, false, noPoll)
	require.NoError(t, err)
	require.Nil(t, got.Points)
}

func TestPlanarFacePenetrationPolls(t *testing.T) {
	box, tray := cornerDown([3]float64{2, 3, -tiny}), faceTray()
	result, err := planar.ClassifyPlanar(&box, &tray, noPoll)
	require.NoError(t, err)
	calls := 0
	_, err = planar.PlanarFacePenetration(&box, &tray, result.Crossings, true, false, func() error {
		calls++
		return nil
	})
	require.NoError(t, err)
	require.Positive(t, calls)
	stop := errors.New("stop")
	for limit := 1; limit <= calls; limit++ {
		n := 0
		_, err := planar.PlanarFacePenetration(&box, &tray, result.Crossings, true, false, func() error {
			if n++; n >= limit {
				return stop
			}
			return nil
		})
		require.ErrorIs(t, err, stop, "limit=%d", limit)
	}
}
