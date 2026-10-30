package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// An inward shell keeps its wall thickness t at the concave boss root: the
// cavity stays at distance t from the root edge, so a point 1.41 mm from that
// edge is material when t = 2. The 60×40×20 box carries a 20×20 boss over
// z ∈ [20, 30]. Below the root, the cavity slice at s = 20 − z is the boss
// rectangle inset by sqrt(t² − s²), which integrates to the transition
// volume 800 − 80π + 64/3. The source holds 52000; the arms differ only in
// their lower and upper inset cavities.
//
// LocatePoint leaves a point in a 2 mm wall undecided on this body: its
// mesh's occupied-volume bound exceeds the ball such a point has room for.
// The probes therefore read the verified 0.05 mm mesh by ray parity, with a
// margin of at least 0.12 mm to the analytic surface.
func TestShellStackedBossKeepsThicknessAtRoot(t *testing.T) {
	t.Parallel()
	band := 800 + 64.0/3 - 80*math.Pi
	for _, tc := range []struct {
		name   string
		remove func(*decad.Body) decad.FaceSelector
		closed bool
		// lower is the 56×36 plate inset over [2, 18], or [0, 18] when the
		// plate bottom is open; upper the 16×16 boss inset over [20, 30],
		// or [20, 28] under a kept roof.
		lower, upper float64
	}{
		{name: "top open", remove: func(b *decad.Body) decad.FaceSelector {
			return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))).Exactly(1)
		}, lower: 56 * 36 * 16, upper: 16 * 16 * 10},
		{name: "bottom open", remove: func(b *decad.Body) decad.FaceSelector {
			return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b))).Exactly(1)
		}, lower: 56 * 36 * 18, upper: 16 * 16 * 8},
		{name: "closed", closed: true, lower: 56 * 36 * 16, upper: 16 * 16 * 8},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stack, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 60, 40, 20),
				boxBodyAtZ(t, doc, 20, 10, 40, 30, 20, 10))
			require.NoError(t, err)
			var shell *decad.Body
			if tc.closed {
				shell, err = decad.Shell(t.Context(), stack, nil, units.Millimeters(2), decad.WithNoOpenings())
			} else {
				shell, err = decad.Shell(t.Context(), stack, tc.remove(stack), units.Millimeters(2))
			}
			require.NoError(t, err)
			volume, err := shell.Volume()
			require.NoError(t, err)
			want := 52000 - tc.lower - tc.upper - band
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
			mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			// 1 mm inside the boss outline and 1 mm below the root.
			require.True(t, meshEncloses(mesh, r3.NewVec(21, 20, 19)), "a point 1.41 mm from the root edge is material")
			// sqrt(2)·1.5 ≈ 2.12 mm from the root edge, in the cavity.
			require.False(t, meshEncloses(mesh, r3.NewVec(21.5, 20, 18.5)), "a point 2.12 mm from the root edge is cavity")
		})
	}
}

// The circular boss keeps t at its root circle in the same way: radius 4 at
// (10, 10), t = 1, a point at radius 3.5 and z = 4.5 lies sqrt(0.5) mm from
// the root circle and is material. The transition volume is
// π(r²t − rπt²/2 + 2t³/3) = 50π/3 − 2π².
func TestShellRoundStackedBossKeepsThicknessAtRoot(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stack := roundBossFromSketch(t, doc, 10, 10)
	shell, err := decad.Shell(t.Context(), stack,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := shell.Volume()
	require.NoError(t, err)
	want := 2000 + 80*math.Pi - 972 - 45*math.Pi - (50*math.Pi/3 - 2*math.Pi*math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.True(t, meshEncloses(mesh, r3.NewVec(13.5, 10, 4.5)), "a point 0.71 mm from the root circle is material")
	// Radius 2.9 at z = 4.1 lies sqrt(2.02) ≈ 1.42 mm from the root circle.
	require.False(t, meshEncloses(mesh, r3.NewVec(12.9, 10, 4.1)), "a point 1.42 mm from the root circle is cavity")
}

// meshEncloses reports whether p lies inside the closed mesh by the parity
// of the triangles a ray from p crosses. The ray's direction has no zero
// component, so for these fixtures it meets no mesh edge or vertex.
func meshEncloses(mesh *decad.Mesh, p r3.Vec) bool {
	verts := mesh.Vertices()
	dir := r3.NewVec(0.2718281828, 0.3141592653, 0.9093)
	crossings := 0
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		e1, e2 := b.Sub(a), c.Sub(a)
		h := dir.Cross(e2)
		det := e1.Dot(h)
		if det == 0 {
			continue
		}
		s := p.Sub(a)
		u := s.Dot(h) / det
		if u < 0 || u > 1 {
			continue
		}
		q := s.Cross(e1)
		v := dir.Dot(q) / det
		if v < 0 || u+v > 1 {
			continue
		}
		if e2.Dot(q)/det > 0 {
			crossings++
		}
	}
	return crossings%2 == 1
}
