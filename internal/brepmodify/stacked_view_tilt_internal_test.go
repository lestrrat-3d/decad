package brepmodify

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// tiltCorners is the counter-clockwise outline of the test record: a parallelogram whose sides 0 and 2 run along
// X and whose sides 1 and 3 are oblique. Side k walks from corner k to corner k+1, and wall(k) is swept along Z.
var tiltCorners = []sectionrecord.Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 14, V: 20}, {U: 4, V: 20}}

// tiltRecord is a parallelogram prism 30 tall stated as a brep record whose section displacement is delta,
// placed by xform: one swept face per side, then the two caps.
func tiltRecord(t *testing.T, delta float64, xform r3.Transform) breprecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	var segs []sectionrecord.CurveSegment
	for i, c := range tiltCorners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: tiltCorners[(i+1)%len(tiltCorners)], TStart: 0, TEnd: 1})
	}
	region := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	bp := breprecord.Payload{Xform: xform}
	for _, seg := range segs {
		bp.Faces = append(bp.Faces, breprecord.Face{Frame: frame, Wall: seg, Z0: 0, Z1: 30, Delta: delta})
	}
	bp.Faces = append(bp.Faces,
		breprecord.Face{Frame: frame, Region: &region, Z0: 0, Z1: 0, Delta: delta},
		breprecord.Face{Frame: frame, Region: &region, Outward: true, Z0: 30, Z1: 30, Delta: delta})
	bp.AssignRoles()
	return bp
}

func buildTilt(t *testing.T, bp breprecord.Payload) *topo.Body {
	t.Helper()
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	body, err := breprecord.Build(t.Context(), env, bp)
	require.NoError(t, err)
	return body
}

// ratTriple reads a vector as exact rationals.
func ratTriple(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

// TestStackedViewTilt measures each swept wall of a displaced record against the wall the record denotes: the
// line through two corners of tiltCorners, swept along Z. Each entry's extent bounds the true corners' distance
// from the tag plane, and its theta bounds the distance between the true unit normal and the tag's.
func TestStackedViewTilt(t *testing.T) {
	t.Parallel()
	const delta = 1e-12
	bp := tiltRecord(t, delta, r3.Identity())
	body := buildTilt(t, bp)
	tilt := stackedViewTilt(body, bp)
	faces := topobuild.Faces(body)

	// All four swept walls are straight and swept along Z, so each has an entry, and no cap does.
	require.Len(t, tilt, 4)
	for k := range 4 {
		got, ok := tilt[k]
		require.True(t, ok, "wall(%d)", k)
		plane := faces[k].Surface.(surfacegeom.Plane)

		requireTiltCovers(t, plane, k, got)
	}

	t.Run("a planar record face has none", func(t *testing.T) {
		t.Parallel()
		// The record states an axis-aligned wall as the planar face(k): the view keeps that plane exactly.
		body := buildTilt(t, bp)
		body.Lumps[0].Shells[0].Faces[0].Origins[0].Role = "face(0)"
		got := stackedViewTilt(body, bp)
		require.NotContains(t, got, 0)
		require.Contains(t, got, 1)
	})
	t.Run("an exact record has none", func(t *testing.T) {
		t.Parallel()
		exact := tiltRecord(t, 0, r3.Identity())
		require.Nil(t, stackedViewTilt(buildTilt(t, exact), exact))
	})
	t.Run("a non-finite displacement has none", func(t *testing.T) {
		t.Parallel()
		got := tiltRecord(t, delta, r3.Identity())
		body := buildTilt(t, got)
		got.CarrierCharge = math.Inf(1)
		require.Nil(t, stackedViewTilt(body, got))
		got.CarrierCharge = math.NaN()
		require.Nil(t, stackedViewTilt(body, got))
	})
	t.Run("a placement that breaks the sweep gate", func(t *testing.T) {
		t.Parallel()
		// Rotating about X by a float angle rounds the wall's normal off the sweep exactly, so n . a is not zero.
		turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(0.3))
		require.NoError(t, err)
		got := tiltRecord(t, delta, turn)
		// The sides along X keep an exact zero (their normal's rounded entries cancel against the axis'), and the
		// oblique sides lose it.
		tilted := stackedViewTilt(buildTilt(t, got), got)
		require.NotContains(t, tilted, 1)
		require.NotContains(t, tilted, 3)
		require.Contains(t, tilted, 0)
	})
	t.Run("a wall no longer than four extents", func(t *testing.T) {
		t.Parallel()
		body := buildTilt(t, bp)
		for _, v := range topobuild.Vertices(body) {
			v.Bound = units.Millimeters(6)
		}
		// The oblique sides are 20.4 long at right angles to the sweep, under four times the bound.
		require.Empty(t, stackedViewTilt(body, bp))
		for _, v := range topobuild.Vertices(body) {
			v.Bound = units.Millimeters(3)
		}
		// The sides along X are 10 long, still under four times it. The oblique ones state theta = sqrt2 * 2E /
		// (L - 2e) with E = e = 3 (the tag passes through the vertices) and L = sqrt(4^2 + 20^2).
		got := stackedViewTilt(body, bp)
		require.Len(t, got, 2)
		want := math.Sqrt2 * 6 / (math.Hypot(4, 20) - 6)
		for _, k := range []int{1, 3} {
			require.Contains(t, got, k)
			require.InDelta(t, want, got[k].theta, 1e-9)
			require.GreaterOrEqual(t, got[k].theta, want*(1-1e-12), "rounded up, never below the exact value")
			require.InDelta(t, 3, got[k].extent, 1e-9)
		}
	})
	t.Run("a tag turned against the record's rule", func(t *testing.T) {
		t.Parallel()
		body := buildTilt(t, bp)
		plane := body.Lumps[0].Shells[0].Faces[1].Surface.(surfacegeom.Plane)
		// Exchanging U and V negates the normal: the tag now faces into the material.
		flipped, err := r3.NewFrame(plane.Frame.Origin(), plane.Frame.V(), plane.Frame.U())
		require.NoError(t, err)
		body.Lumps[0].Shells[0].Faces[1].Surface = surfacegeom.Plane{Frame: flipped}
		got := stackedViewTilt(body, bp)
		require.NotContains(t, got, 1)
		require.Contains(t, got, 0)
	})
	t.Run("a reflected placement keeps the orientation rule", func(t *testing.T) {
		t.Parallel()
		mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		flip, err := r3.Reflection(mirror)
		require.NoError(t, err)
		require.True(t, flip.IsReflection())
		got := tiltRecord(t, delta, flip)
		body := buildTilt(t, got)
		require.Len(t, stackedViewTilt(body, got), 4)
	})
	t.Run("a tag tilted by about thirty degrees keeps the claim", func(t *testing.T) {
		t.Parallel()
		// Turning wall(1)'s tag about Z through its own midpoint moves its two ends to +-(L/2) sin(a) from it, a
		// tilt near the largest the gate L > 4E admits, where the two unit normals are farther apart than the
		// sine that E and L bound: only the root-two factor covers it.
		body := buildTilt(t, bp)
		face := body.Lumps[0].Shells[0].Faces[1]
		plane := face.Surface.(surfacegeom.Plane)
		const a = 29 * math.Pi / 180
		u, v := plane.Frame.U(), plane.Frame.V()
		turned, err := r3.NewFrame(plane.Frame.Origin(), u.Scale(math.Cos(a)).Add(plane.Frame.N().Scale(-math.Sin(a))), v)
		require.NoError(t, err)
		face.Surface = surfacegeom.Plane{Frame: turned}
		got := stackedViewTilt(body, bp)
		require.Contains(t, got, 1)
		requireTiltCovers(t, surfacegeom.Plane{Frame: turned}, 1, got[1])
		require.Greater(t, got[1].theta, 0.4, "a tilt of 29 degrees is stated, not rounded away")
	})
	t.Run("a face with no walk order", func(t *testing.T) {
		t.Parallel()
		// Moving a side line first reads the loop out of the record's order: the first coedge is not a rim. The
		// side line's ends differ across the wall only by rounding, however it is turned, so the rim test refuses.
		for _, sign := range []float64{1, -1} {
			body := buildTilt(t, bp)
			loop := body.Lumps[0].Shells[0].Faces[1].Loops[0]
			loop.CoEdges = append(loop.CoEdges[1:], loop.CoEdges[0])
			side := loop.CoEdges[0].Edge
			require.NotEqual(t, side.Start.Position.Z, side.End.Position.Z, "the first coedge is a side line")
			// Nudge one end along the wall, inside the vertices' bound, so the orientation sign alone could
			// read either way.
			d := r3.NewVec(4, 20, 0).Scale(sign * 1e-13 / math.Hypot(4, 20))
			side.End.Position = side.End.Position.Add(d)
			require.NotContains(t, stackedViewTilt(body, bp), 1, "sign %v", sign)
		}
	})
}

// requireTiltCovers checks got against the wall the record denotes for wall(k): the line through two corners of
// tiltCorners, swept along Z. The extent bounds each true corner's distance from the tag plane, exactly, and theta
// bounds the distance between the true outward unit normal (dy, -dx) / |d| and the tag's, to 300 bits.
func requireTiltCovers(t *testing.T, plane surfacegeom.Plane, k int, got viewTilt) {
	t.Helper()
	n, o := ratTriple(plane.Frame.N()), ratTriple(plane.Frame.Origin())
	nn := new(big.Rat)
	for i := range n {
		nn.Add(nn, new(big.Rat).Mul(n[i], n[i]))
	}
	extent2 := new(big.Rat).Mul(ratOf(got.extent), ratOf(got.extent))
	for _, c := range []sectionrecord.Point2{tiltCorners[k], tiltCorners[(k+1)%4]} {
		p := [3]*big.Rat{ratOf(c.U), ratOf(c.V), new(big.Rat)}
		off := new(big.Rat)
		for i := range n {
			off.Add(off, new(big.Rat).Mul(n[i], new(big.Rat).Sub(p[i], o[i])))
		}
		require.LessOrEqual(t, new(big.Rat).Mul(off, off).Cmp(new(big.Rat).Mul(extent2, nn)), 0,
			"wall(%d): the true corner lies farther from the tag than the stated extent", k)
	}

	a, b := tiltCorners[k], tiltCorners[(k+1)%4]
	prec := func() *big.Float { return new(big.Float).SetPrec(300) }
	dx, dy := prec().SetFloat64(b.U-a.U), prec().SetFloat64(b.V-a.V)
	length := prec().Sqrt(prec().Add(prec().Mul(dx, dx), prec().Mul(dy, dy)))
	tag := plane.Frame.N()
	dist2 := prec()
	for _, pair := range [][2]*big.Float{
		{prec().Quo(dy, length), prec().SetFloat64(tag.X)},
		{prec().Neg(prec().Quo(dx, length)), prec().SetFloat64(tag.Y)},
	} {
		diff := prec().Sub(pair[0], pair[1])
		dist2.Add(dist2, prec().Mul(diff, diff))
	}
	theta := prec().SetFloat64(got.theta)
	require.LessOrEqual(t, dist2.Cmp(prec().Mul(theta, theta)), 0,
		"wall(%d): the true normal is farther from the tag's than theta", k)
	require.Less(t, got.extent, 100.0)
	require.Greater(t, got.extent, 0.0)
	require.Greater(t, got.theta, 0.0)
}

// TestApplyViewTilt states each tilt on a retained Plane group that states no departure, and untags every other
// group in the map.
func TestApplyViewTilt(t *testing.T) {
	t.Parallel()
	plane := surfacegeom.Plane{}
	retained := func(surface surfacegeom.Surface) facetedrecord.Group {
		return facetedrecord.Group{Surface: surface, Retained: true}
	}
	groups := []facetedrecord.Group{
		retained(plane),
		retained(surfacegeom.Cylinder{}),
		{Surface: plane},
		{Surface: plane, Retained: true, NormalBound: 1e-3, ExtentDelta: 7},
		retained(plane),
	}
	require.True(t, applyViewTilt(groups, map[int]viewTilt{
		0: {theta: 3e-14, extent: 5e-15}, 1: {theta: 1, extent: 1}, 2: {theta: 1, extent: 1}, 3: {theta: 1, extent: 1}}))
	require.Equal(t, 3e-14, groups[0].NormalBound)
	require.Equal(t, 5e-15, groups[0].ExtentDelta)
	require.Equal(t, plane, groups[0].Surface)
	require.True(t, groups[0].Retained)
	for _, i := range []int{1, 2, 3} {
		require.Nil(t, groups[i].Surface, "group %d", i)
		require.False(t, groups[i].Retained)
		require.Zero(t, groups[i].NormalBound)
		require.Zero(t, groups[i].ExtentDelta)
	}
	require.Zero(t, groups[4].ExtentDelta, "a group outside the map is left alone")
	require.False(t, applyViewTilt(groups, map[int]viewTilt{5: {}}))
	require.False(t, applyViewTilt(groups, map[int]viewTilt{-1: {}}))
}
