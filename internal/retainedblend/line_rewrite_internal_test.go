package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestLineEndFaceTurn reads an end face G at a straight edge's corner V = 0 in the
// plane z = 0. A's chain leaves V toward (5, 0) and B's toward (0, 5); the
// rows (1, 0) and (0, 1) are the blend's setback points.
func TestLineEndFaceTurn(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	verts := []r3.Vec{{}, r3.NewVec(5, 0, 0), r3.NewVec(5, 5, 0), r3.NewVec(0, 5, 0), x, y}
	end := lineEnd{g: plane{n: z}, v: 0}
	// G convex at V holds the quadrant between the chains; G reflex at V holds
	// the other three, and its loop runs the other way.
	convexG := faceLoops{loops: [][]int{{0, 1, 2, 3}}}
	reflexG := faceLoops{loops: [][]int{{0, 3, 2, 1}}}
	read := func(g faceLoops, dirs *[2]r3.Vec) (bool, error) {
		w := &work{verts: verts, beta: make([]float64, len(verts))}
		_, takes, err := w.endFace(end, g, []int{1}, []int{3}, [2]sideKind{trims, trims}, nil, nil, []int{4, 5},
			[]r3.Vec{x, y}, 1e-9, true, dirs)
		return takes, err
	}

	t.Run("a convex end face gives up the corner", func(t *testing.T) {
		t.Parallel()
		takes, err := read(convexG, &[2]r3.Vec{x, y})
		require.NoError(t, err)
		require.False(t, takes)
	})

	t.Run("a reflex end face takes the cap on the straight-edge route only", func(t *testing.T) {
		t.Parallel()
		takes, err := read(reflexG, &[2]r3.Vec{x, y})
		require.NoError(t, err)
		require.True(t, takes)
		_, err = read(reflexG, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's corner does not match the edge's convexity")
	})

	t.Run("an end where the wall extends takes the wall's run and the corner", func(t *testing.T) {
		t.Parallel()
		// The cap lies below A's chain: the rows run from ta = (1, 0) to the
		// wall's setback point (0.5, -0.8), and the wall's cut in G's plane
		// runs on through (0.2, -0.6) to V. G's loop keeps V and gains the
		// cap.
		capped := append(append([]r3.Vec(nil), verts...), r3.NewVec(0.5, -0.8, 0), r3.NewVec(0.2, -0.6, 0))
		read := func(dirs *[2]r3.Vec) ([][]int, error) {
			w := &work{verts: capped, beta: make([]float64, len(capped))}
			loops, takes, err := w.endFaceRun(end, convexG, []int{1}, []int{3}, [2]sideKind{trims, extends}, nil, nil,
				[]int{4, 6}, []int{7}, []r3.Vec{x, capped[6]}, 1e-9, true, dirs)
			require.False(t, takes)
			return loops, err
		}
		loops, err := read(&[2]r3.Vec{x, y})
		require.NoError(t, err)
		require.Len(t, loops, 1)
		at := 0
		for loops[0][at] != 1 {
			at++
		}
		got := append(append([]int(nil), loops[0][at:]...), loops[0][:at]...)
		require.Equal(t, []int{1, 2, 3, 0, 7, 6, 4}, got)

		// The held vertices show a square convex corner; the tag directions
		// decide the turn.
		near, ok := r3.NewVec(-1, 1e-4, 0).Normalize()
		require.True(t, ok)
		_, err = read(&[2]r3.Vec{x, near})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's turn at the edge's corner is undecided")
		_, err = read(&[2]r3.Vec{y, x})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's corner does not match the edge's convexity")
	})

	t.Run("a turn within the bound reads undecided and refuses", func(t *testing.T) {
		t.Parallel()
		// The held vertices show a square corner, but the tag lines meet at a
		// sine of 1e-4, under minSine: the turn is read from the tags alone.
		near, ok := r3.NewVec(-1, 1e-4, 0).Normalize()
		require.True(t, ok)
		_, err := read(convexG, &[2]r3.Vec{x, near})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's turn at the edge's corner is undecided")
	})
}

// TestRowStepEndPlaneSagitta projects a fillet's rows along the edge d = x onto
// an end plane through the axis with |d·n| = c = 1/2. The plane's ellipse is the
// cross-section's circle of radius r stretched by 1/c along u = y, so a chord of
// half-angle h centred on u keeps its direction while its sagitta r·(1 − cos h)
// stretches to r·(1 − cos h)/c; no chord's grows more.
func TestRowStepEndPlaneSagitta(t *testing.T) {
	t.Parallel()
	const r, c = 2.0, 0.5
	tau := math.Ldexp(r, -10)
	d := r3.NewVec(1, 0, 0)
	g := plane{n: r3.NewVec(c, math.Sqrt(1-c*c), 0)}
	circle := func(beta float64) r3.Vec { return r3.NewVec(0, r*math.Cos(beta), r*math.Sin(beta)) }
	// inPlane is the largest distance in the end plane between the arc of
	// half-angle h about beta and its chord, each projected along d.
	inPlane := func(beta, h float64) float64 {
		p0, p1 := setbackPoint(circle(beta-h), d, g), setbackPoint(circle(beta+h), d, g)
		worst := 0.0
		for k := range 201 {
			q := setbackPoint(circle(beta-h+2*h*float64(k)/200), d, g)
			worst = math.Max(worst, distToSegment(q, p0, p1))
		}
		return worst
	}
	sagitta := func(h float64) float64 { return r * (1 - math.Cos(h)) / c }

	require.Equal(t, c, endSine(1, d, plane{n: r3.NewVec(-0.8, 0.6, 0)}, g, plane{n: d}),
		"the shallowest plane sets the sine")
	require.Equal(t, 1.0, endSine(1, d, plane{n: d}))

	for _, tc := range []struct {
		name string
		c    float64
		over bool
	}{
		// Spaced for the cross-section alone, the chord on u reaches tau/c.
		{"rows spaced for the cross-section", 1, true},
		{"rows spaced for the end plane", endSine(1, d, g), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			h := rowStep(tau, r, tc.c) / 2
			want := sagitta(h)
			require.InEpsilon(t, want, inPlane(0, h), 1e-9, "the chord on u")
			for k := range 64 {
				beta := 2 * math.Pi * float64(k) / 64
				require.LessOrEqual(t, inPlane(beta, h), want*(1+1e-9), "the chord at %v", beta)
			}
			if tc.over {
				require.InEpsilon(t, tau/c, want, 1e-9)
				return
			}
			require.InEpsilon(t, tau, want, 1e-9)
			require.LessOrEqual(t, want, tau*(1+1e-12))
		})
	}
}
