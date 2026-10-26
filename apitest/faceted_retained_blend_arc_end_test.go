package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-modify-reach-design.md §12 for increment
// M4b: an open tangent chain whose first edge is an arc that ends on a plane
// parallel to its axis. The rounded-corner box (40x20x20, corner radius 3 about
// (3,3)) loses everything below x = 1.5 to a placed block, so the top outline
// is the arc from the new face x = 1.5 to the tangent point (3,0), then the
// straight edge to (40,0). Their line is 37 mm long and the arc spans π/6.

const (
	arcEndLine = 37.0
	arcEndGx   = 1.5
)

// arcEndBody returns the trimmed box and the seed arc at the top.
func arcEndBody(t *testing.T) (*decad.Body, decad.EdgeSelector) {
	t.Helper()
	sc := newRetentionScene()
	corner := roundedCornerBox(t, sc)
	block := placed(t, rectBox(t, sc.w, sc.doc, -5, -5, arcEndGx, 25, 30), -5)
	trimmed, err := decad.Cut(t.Context(), corner, block)
	require.NoError(t, err)
	return trimmed, arcSeedAt(t, trimmed, 20)
}

// arcEndSpan is the angle the arc covers when G lies at distance h from the
// axis of the corner cylinder of radius R, from the tangent point to G.
func arcEndSpan(rho float64) float64 { return math.Pi/2 - math.Acos(arcEndGx/rho) }

// arcEndRemoved is the volume a convex blend takes along the arc and the line.
// inside the corner section [R-d, R] x [-d, 0] the removed region is the part
// for which keep(rho, z) holds; it sweeps arcEndSpan(rho) about the axis.
func arcEndRemoved(d, line float64, outside func(dr, dz float64) bool) float64 {
	const n, R = 1500, openBoxR
	sum := 0.0
	for i := range n {
		rho := R - d + d*(float64(i)+0.5)/n
		for j := range n {
			z := -d + d*(float64(j)+0.5)/n
			if outside(rho-(R-d), z+d) {
				sum += arcEndSpan(rho) * rho * (d / n) * (d / n)
			}
		}
	}
	af := 0.0
	for i := range n {
		for j := range n {
			x := d * (float64(i) + 0.5) / n
			y := d * (float64(j) + 0.5) / n
			if outside(x, y) {
				af += (d / n) * (d / n)
			}
		}
	}
	return sum + af*line
}

func TestFacetedRetainedBlendArcEnd(t *testing.T) {
	t.Parallel()

	t.Run("fillet of a chain that ends on an arc", func(t *testing.T) {
		t.Parallel()
		body, seed := arcEndBody(t)
		before, err := body.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), body, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		want := arcEndRemoved(r, arcEndLine, func(dr, dz float64) bool { return dr*dr+dz*dz > r*r })
		requireVolumeChange(t, before, after, want)
		requireVerifiedMesh(t, out)
	})

	t.Run("the fillet's blend kinds and junction and end curves", func(t *testing.T) {
		t.Parallel()
		body, seed := arcEndBody(t)
		const r = 0.5
		out, err := decad.Fillet(t.Context(), body, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 1, cylinders)
		require.Equal(t, 1, tori)
		require.InDelta(t, openBoxR-r, majors[0], 1e-12)
		// The junction at x = 3 joins the cylinder and the torus in a quarter
		// circle about (3,0.5,19.5); the straight edge's own end on x = 40 is the
		// other. The torus meets x = 1.5 in a chain of points, not an arc.
		want := map[r3.Vec]bool{r3.NewVec(3, 0.5, 19.5): true, r3.NewVec(40, 0.5, 19.5): true}
		found := 0
		for _, e := range out.Edges() {
			arc, ok := e.Curve().(decad.Arc3)
			if !ok || math.Abs(arc.Radius.Base()-r) > 1e-12 {
				continue
			}
			found++
			ok = false
			for c := range want {
				ok = ok || arc.Center.Sub(c).Len() < 1e-9
			}
			require.True(t, ok, "an unexpected arc about %v", arc.Center)
			require.InDelta(t, 1, math.Abs(arc.Axis.X), 1e-12)
		}
		require.Equal(t, 2, found)
		requireVerifiedMesh(t, out)
	})

	t.Run("the bottom outline runs the other way along its face loop", func(t *testing.T) {
		t.Parallel()
		body, _ := arcEndBody(t)
		before, err := body.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), body, arcSeedAt(t, body, 0), units.Millimeters(r),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		want := arcEndRemoved(r, arcEndLine, func(dr, dz float64) bool { return dr*dr+dz*dz > r*r })
		requireVolumeChange(t, before, after, want)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer of a chain that ends on an arc", func(t *testing.T) {
		t.Parallel()
		body, seed := arcEndBody(t)
		before, err := body.Volume()
		require.NoError(t, err)
		const d = 1.0
		out, err := decad.Chamfer(t.Context(), body, seed, units.Millimeters(d), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		want := arcEndRemoved(d, arcEndLine, func(dr, dz float64) bool { return dr+dz > d })
		requireVolumeChange(t, before, after, want)
		cones, planes := 0, 0
		for _, f := range out.Faces() {
			switch f.Surface().(type) {
			case decad.Cone:
				cones++
			case decad.Plane:
				planes++
			}
		}
		require.Equal(t, 1, cones)
		require.Equal(t, 7, planes, "the line's bevel and the six faces of the trimmed box")
		requireVerifiedMesh(t, out)
	})

	t.Run("the surface area changes by the exact bands, blend faces and end corners", func(t *testing.T) {
		t.Parallel()
		body, seed := arcEndBody(t)
		before, err := body.Area()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), body, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Area()
		require.NoError(t, err)
		requireAreaChange(t, before, after, arcEndFilletArea(r))
	})

	t.Run("a setback circle that misses the end face is refused", func(t *testing.T) {
		t.Parallel()
		// The end face lies 1.5 mm from the axis, so the plane's setback circle
		// of radius 3 - 2 = 1 never reaches it.
		body, seed := arcEndBody(t)
		_, err := decad.Fillet(t.Context(), body, seed, units.Millimeters(2), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "does not reach an end face")
	})
}

// arcEndFilletArea is the exact change of the surface area of arcEndBody
// under a fillet of radius r along its top outline, by independent sums: the
// straight edge's bands and quarter cylinder less its end corner, and the
// arc's torus patch less its two bands and the corner of the end face.
func arcEndFilletArea(r float64) float64 {
	const n, R = 20000, openBoxR
	af := r * r * (1 - math.Pi/4)
	line := arcEndLine*(r*math.Pi/2-2*r) - af
	var blend, base float64
	for i := range n {
		a := math.Pi / 2 * (float64(i) + 0.5) / n
		rho := R - r*(1-math.Sin(a))
		blend += r * (math.Pi / 2 / n) * rho * arcEndSpan(rho)
		rb := R - r + r*(float64(i)+0.5)/n
		base += (r / n) * rb * arcEndSpan(rb)
	}
	wall := R * arcEndSpan(R) * r
	// The end face x = 1.5: the corner region between the end curve and the
	// two lines through the corner, in (y, z).
	y := func(rho float64) float64 { return R - math.Sqrt(rho*rho-arcEndGx*arcEndGx) }
	pts := [][2]float64{{y(R), 0}}
	for i := 0; i <= n; i++ {
		a := math.Pi / 2 * float64(i) / n
		pts = append(pts, [2]float64{y(R - r*(1-math.Sin(a))), -r * (1 - math.Cos(a))})
	}
	corner := 0.0
	for i := range pts {
		q := pts[(i+1)%len(pts)]
		corner += pts[i][0]*q[1] - q[0]*pts[i][1]
	}
	return line + blend - base - wall - math.Abs(corner)/2
}
