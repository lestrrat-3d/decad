package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// A post of radius 3 stands 100 mm tall on the XY plane. Six struts of radius
// 0.8 rise into it at 60 degrees from one side, 12 mm apart, each joined in
// turn from the top down. Every strut's rim lands on the post's wall facets,
// which run the whole post height, below the rims the earlier struts left.
const (
	slabPostRadius  = 3.0
	slabPostHeight  = 100.0
	slabStrutRadius = 0.8
	slabStrutLength = 12.0
	slabStrutRise   = 60.0
	slabStrutCount  = 6
	slabStrutPitch  = 12.0
	slabStrutTop    = 90.0
)

// slabCylinder extrudes a circle of radius r about the origin of the plane
// through origin with axes u and v, length mm along u × v.
func slabCylinder(t *testing.T, doc *decad.Document, origin, u, v r3.Vec, r, length float64) *decad.Body {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), r), r))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// slabStrut is the strut whose axis ends on the post axis at height z, rising
// at slabStrutRise toward it from +X.
func slabStrut(t *testing.T, doc *decad.Document, z float64) *decad.Body {
	t.Helper()
	sin, cos := math.Sincos(slabStrutRise * math.Pi / 180)
	axis := r3.NewVec(-cos, 0, sin)
	start := r3.NewVec(0, 0, z).Sub(axis.Scale(slabStrutLength))
	u := r3.NewVec(0, 1, 0)
	return slabCylinder(t, doc, start, u, axis.Cross(u), slabStrutRadius, slabStrutLength)
}

// slabMeshFloor reads the smallest tolerance body tessellates at.
func slabMeshFloor(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	_, err := body.Tessellate(t.Context(), units.Millimeters(1e-9))
	var floor *decad.MeshBoundError
	require.True(t, errors.As(err, &floor), "%v", err)
	return floor.Floor().Base()
}

// Each strut's rim reads the post wall's own chord departure, not the bound of
// the rim an earlier strut left on the same wall facets: the post's wall is a
// retained Cylinder, and the cylinder lies within its chord's sagitta of every
// wall facet's plane however large a corner's bound. So the body after the
// sixth strut holds its mesh within 1.5 times the bound after the first, where
// each strut would otherwise amplify the earlier rim's bound by the crossing's
// 1/sin and reach about ten times it, and it tessellates at 0.05 mm with every
// verification on and no WithRimSplit.
func TestCylinderWallChainKeepsEachRimBound(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	post := slabCylinder(t, doc, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), slabPostRadius, slabPostHeight)
	postVolume, err := post.Volume()
	require.NoError(t, err)

	body := post
	var first float64
	for i := range slabStrutCount {
		body, err = decad.Union(body, slabStrut(t, doc, slabStrutTop-float64(i)*slabStrutPitch)).Do(t.Context())
		require.NoError(t, err, "strut %d", i)
		if i == 0 {
			first = slabMeshFloor(t, body)
		}
	}
	last := slabMeshFloor(t, body)
	require.LessOrEqual(t, last, 1.5*first, "first %v last %v", first, last)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), units.Millimeters(0.05).Base())

	// Each strut adds the part of its volume outside the post, which is less
	// than its own volume and more than nothing.
	volume, err := body.Volume()
	require.NoError(t, err)
	strut := units.CubicMillimeters(math.Pi * slabStrutRadius * slabStrutRadius * slabStrutLength).Base()
	added := volume.Value.Base() - postVolume.Value.Base()
	slack := volume.Bound.Base() + postVolume.Bound.Base()
	require.Greater(t, added, slack)
	require.Less(t, added, slabStrutCount*strut-slack)

	sin, cos := math.Sincos(slabStrutRise * math.Pi / 180)
	for i := range slabStrutCount {
		end := r3.NewVec(0, 0, slabStrutTop-float64(i)*slabStrutPitch)
		mid := end.Sub(r3.NewVec(-cos, 0, sin).Scale(slabStrutLength * 0.75))
		at, err := body.LocatePoint(t.Context(), mid, units.Millimeters(0.05))
		require.NoError(t, err)
		require.Equal(t, decad.PointInside, at, "strut %d outside the post", i)
	}
}
